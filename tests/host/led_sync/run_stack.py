#!/usr/bin/env python3
"""Replay optimized nRF52 LED workers and their linked PWM/WS2812 drivers.

    uv run --with unicorn --with pyelftools run_stack.py APP/zephyr.elf

The neighboring .config is required. DWARF supplies the linked usable thread
budget and every fixture field offset; missing metadata is an error. Production
led_thread (including inlined policy/adapter), driver calls, pinctrl, PM and
cbprintf packaging execute unchanged. Driver boot init uses a separate stack.

Kernel scheduling/semaphore/mutex boundaries are modeled. Real nonblocking slab
allocation/free and message-queue put/get execute, with independent ownership
checks. DMA completion is modeled at the driver's sleep boundary, excluding its
asynchronous IRQ stack. The HF clock fixture is already ON with one external reference, so actual
onoff_request executes its synchronous callback and actual start_transfer/nrfx
code runs on the worker. PWM MMIO STOP produces the STOPPED event and models its
asynchronous control-block update (the IRQ itself is excluded). Error cases
inject a peripheral initialization-state failure at START or an invalid PWM
channel at the API boundary; actual driver recovery and log packaging execute.
Message queue backpressure is not injected: sole-owner reachability is unproved.

This measures a LOWER BOUND, not maximum or physical safety. Thread-entry wrapper,
exception/context frames, interrupts, contention, real clock transitions, kernel
leaves, logging allocation/finalization and physical peripheral timing are not
proved. A measured overrun is a failure even though those costs are excluded.
Recent-entry traces are deliberately not described as active backtraces.
"""

import argparse
from collections import Counter, deque
import hashlib
import json
from pathlib import Path

from elftools.elf.elffile import ELFFile
from unicorn import (Uc, UcError, UC_ARCH_ARM, UC_HOOK_CODE, UC_HOOK_MEM_WRITE,
                     UC_MODE_MCLASS, UC_MODE_THUMB)
from unicorn.arm_const import (UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0,
                               UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3,
                               UC_ARM_REG_SP, UC_ARM_REG_XPSR)

SCRATCH = 0x21000000
SENTINEL = SCRATCH + 0xFF000
SETUP_TOP = SCRATCH + 0xFE000
LIMIT = 5_000_000
REGS = (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)

ABI_TYPES = {
    '_static_thread_data', 'device', 'device_state', 'led_engine', 'led_behavior',
    'led_connection_facts', 'led_pending_event', 'led_preview_record',
    'led_owner_state', 'led_selection', 'led_rgb', 'pwm_nrfx_data',
    'i2s_nrfx_drv_data', 'nrfx_i2s_t', 'nrfx_i2s_control_block_t',
    'nrfx_i2s_buffers_t', 'onoff_manager', 'k_msgq', 'k_mem_slab',
    'k_mem_slab_info', 'NRF_I2S_Type', 'led_hardware_info',
    'nrfx_pwm_t', 'nrfx_pwm_control_block_t',
}


def canonical(name):
    return name.split('.')[0]


class Image:
    def __init__(self, path):
        self.path = path
        self.sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
        config = path.with_name('.config')
        if not config.exists():
            config = path.with_name('zephyr.config')
        self.config = dict(line.split('=', 1) for line in config.read_text().splitlines()
                           if line.startswith('CONFIG_') and '=' in line)
        if self.config.get('CONFIG_SOC_SERIES_NRF52') != 'y':
            raise ValueError('Only nRF52 Cortex-M images are supported')
        self.structs, self.enums, self.symbols, self.functions = {}, {}, {}, {}
        self.inline_ranges = []
        with path.open('rb') as stream:
            elf = ELFFile(stream)
            if elf.elfclass != 32 or not elf.little_endian or elf['e_machine'] != 'EM_ARM':
                raise ValueError('Expected little-endian 32-bit ARM ELF')
            table = elf.get_section_by_name('.symtab')
            if table is None or not elf.has_dwarf_info():
                raise ValueError('Unstripped ELF with DWARF is required')
            for symbol in table.iter_symbols():
                if symbol['st_shndx'] == 'SHN_UNDEF':
                    continue
                address, size = int(symbol['st_value']), int(symbol['st_size'])
                self.symbols[symbol.name] = (address, size)
                if symbol['st_info']['type'] == 'STT_FUNC':
                    self.functions[address & ~1] = canonical(symbol.name)
            self.sections = [(int(s['sh_addr']), s.data()) for s in elf.iter_sections()
                             if s['sh_flags'] & 2 and s['sh_type'] != 'SHT_NOBITS'
                             and s['sh_size']]
            for unit in elf.get_dwarf_info().iter_CUs():
                dwarf = elf.get_dwarf_info()
                for die in unit.iter_DIEs():
                    name = die.attributes.get('DW_AT_name')
                    if name and name.value.decode() in ABI_TYPES and die.tag in (
                            'DW_TAG_structure_type', 'DW_TAG_union_type', 'DW_TAG_typedef'):
                        resolved = self.resolve(die)
                        if 'DW_AT_byte_size' not in resolved.attributes:
                            continue
                        fields = self.members(resolved)
                        if fields:
                            key = name.value.decode()
                            old = self.structs.get(key)
                            value = (resolved.attributes['DW_AT_byte_size'].value, fields)
                            if old and old != value:
                                raise ValueError(f'Conflicting DWARF layout: {key}')
                            self.structs[key] = value
                    elif die.tag == 'DW_TAG_enumerator' and name:
                        self.enums[name.value.decode()] = die.attributes['DW_AT_const_value'].value
                    elif die.tag == 'DW_TAG_inlined_subroutine':
                        origin = die
                        while 'DW_AT_abstract_origin' in origin.attributes:
                            origin = origin.get_DIE_from_attribute('DW_AT_abstract_origin')
                        inline_name = origin.attributes.get('DW_AT_name')
                        if not inline_name or inline_name.value.decode() not in (
                                'nrfx_i2s_start', 'nrfx_i2s_init', 'nrfx_pwm_simple_playback',
                                'led_hw_write', 'led_hw_init', 'led_worker_step', 'trigger_start',
                                'pwm_resume', 'pwm_suspend'):
                            continue
                        low = die.attributes.get('DW_AT_low_pc')
                        high = die.attributes.get('DW_AT_high_pc')
                        if low and high:
                            end = high.value if high.form == 'DW_FORM_addr' else low.value + high.value
                            self.inline_ranges.append((low.value, end, inline_name.value.decode()))
                        elif 'DW_AT_ranges' in die.attributes:
                            base = unit.get_top_DIE().attributes.get('DW_AT_low_pc')
                            base = base.value if base else 0
                            entries = dwarf.range_lists().get_range_list_at_offset(
                                die.attributes['DW_AT_ranges'].value, cu=unit)
                            for entry in entries:
                                if hasattr(entry, 'base_address'):
                                    base = entry.base_address
                                else:
                                    self.inline_ranges.append((
                                        entry.begin_offset + (0 if entry.is_absolute else base),
                                        entry.end_offset + (0 if entry.is_absolute else base),
                                        inline_name.value.decode()))

    @staticmethod
    def resolve(die):
        while die.tag in ('DW_TAG_typedef', 'DW_TAG_const_type', 'DW_TAG_volatile_type'):
            die = die.get_DIE_from_attribute('DW_AT_type')
        return die

    @classmethod
    def type_size(cls, die):
        die = cls.resolve(die)
        size = die.attributes.get('DW_AT_byte_size')
        if size:
            return size.value
        if die.tag == 'DW_TAG_pointer_type':
            return 4
        if die.tag == 'DW_TAG_array_type':
            count = 1
            for dimension in die.iter_children():
                if dimension.tag != 'DW_TAG_subrange_type':
                    continue
                upper = dimension.attributes.get('DW_AT_upper_bound')
                explicit = dimension.attributes.get('DW_AT_count')
                lower = dimension.attributes.get('DW_AT_lower_bound')
                count *= explicit.value if explicit else (
                    upper.value - (lower.value if lower else 0) + 1 if upper else 0)
            return count * cls.type_size(die.get_DIE_from_attribute('DW_AT_type'))
        raise ValueError(f'Unsupported DWARF field type: {die.tag}')

    @classmethod
    def members(cls, die, base=0):
        result = {}
        for member in die.iter_children():
            if member.tag != 'DW_TAG_member':
                continue
            location = member.attributes.get('DW_AT_data_member_location')
            if location is None:
                offset = 0
            elif isinstance(location.value, int):
                offset = location.value
            else:
                raise ValueError('Nonconstant DWARF field offset')
            name = member.attributes.get('DW_AT_name')
            field_type = member.get_DIE_from_attribute('DW_AT_type')
            if name is None:
                result.update(cls.members(cls.resolve(field_type), base + offset))
                continue
            size = cls.type_size(field_type)
            bits = member.attributes.get('DW_AT_bit_size')
            bit_offset = member.attributes.get('DW_AT_bit_offset')
            data_bits = member.attributes.get('DW_AT_data_bit_offset')
            shift = size * 8 - bit_offset.value - bits.value if bits and bit_offset else 0
            if data_bits:
                offset = data_bits.value // 8
                shift = data_bits.value % 8
            result[name.value.decode()] = (base + offset, size,
                                          bits.value if bits else size * 8, shift)
        return result

    def symbol(self, name):
        hits = {value for key, value in self.symbols.items() if canonical(key) == name}
        if len(hits) != 1:
            raise ValueError(f'Expected one linked symbol {name}, got {hits}')
        return hits.pop()

    def field(self, kind, name):
        return self.structs[kind][1][name]

    def enum(self, name):
        return self.enums[name]


class Replay:
    def __init__(self, image, case):
        self.image, self.case = image, case
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for start in (0, 0x10000000, 0x20000000, 0x40000000, 0x50000000, 0xE0000000, SCRATCH):
            self.cpu.mem_map(start, 0x100000)
        for address, data in image.sections:
            self.cpu.mem_write(address, data)
        self.measuring = False
        self.now, self.phase, self.instructions = 0, 0, 0
        self.visits, self.stubs = Counter(), Counter()
        self.trace, self.deepest = deque(maxlen=24), []
        self.phase_visits, self.phase_outputs, self.semantics = Counter(), Counter(), set()
        self.phase_injected, self.phase_pixel_frames = Counter(), {}
        self.phase_nonzero_outputs = Counter()
        self.phase_lowest, self.phase_errors, self.phase_previews = {}, {}, {}
        self.phase_powered = {}
        self.phase_semantics = {}
        self.inline_visits, self.inline_seen = Counter(), set()
        self.slabs, self.inflight, self.pending_alloc = {}, set(), []
        self.log_cursor, self.finalized, self.packages, self.conversions = SCRATCH + 0x1000, 0, 0, 0
        self.injected, self.completed, self.error = 0, False, None
        self.pwm_channels, self.pm_actions = set(), Counter()
        self.i2s_commands = Counter()
        self.outputs, self.nonzero_outputs = 0, 0
        self.pwm_regs = {}
        self.i2s_device, self.i2s_data, self.pixel = None, None, None
        self.clock = SCRATCH + 0x800
        self.cpu.hook_add(UC_HOOK_CODE, self.hook)
        self.cpu.hook_add(UC_HOOK_MEM_WRITE, self.mmio)
        self.stack, self.storage = image.symbol('_k_thread_stack_led_thread_id')
        thread = image.symbol('_k_thread_data_led_thread_id')[0]
        self.budget = self.get(thread, '_static_thread_data', 'init_stack_size')
        self.entry = self.get(thread, '_static_thread_data', 'init_entry')
        if (self.get(thread, '_static_thread_data', 'init_stack') != self.stack
                or self.entry != image.symbol('led_thread')[0]
                or not 0 < self.budget <= self.storage):
            raise ValueError('Inconsistent linked LED thread initializer')
        self.top, self.lowest = self.stack + self.storage, self.stack + self.storage
        self.engine = image.symbol('engine')[0]
        self.behaviors, self.behaviors_size = image.symbol('behaviors')
        self.semantic_behaviors, self.semantic_behaviors_size = image.symbol('semantic_behaviors')
        self.debug = image.config.get('CONFIG_LED_DEBUG') == 'y'
        self.kind = 'pixel' if image.config.get('CONFIG_WS2812_STRIP_I2S') == 'y' else 'pwm'
        self.phases = ['startup', 'ready', 'charging', 'charging_fade', 'charging_same_slot', 'charging_down',
                       'event', 'event_gap1', 'event_second', 'event_gap2', 'event_third',
                       'event_gap3', 'event_fourth', 'event_tail', 'event_tail_end', 'event_finish',
                       'stage', 'stage_gap', 'stage_tail', 'stage_finish',
                       'refusal', 'refusal_gap', 'refusal_second', 'refusal_tail', 'refusal_finish',
                       'button_hold', 'button_plateau', 'button_marker', 'button_fade', 'button_down',
                       'button_black', 'button_blink', 'button_blink_gap', 'manual_exit',
                       'cancel', 'cancel_gap', 'cancel_finish',
                       'fault', 'fault_gap', 'fault_second', 'fault_pause', 'fault_repeat', 'fault_repeat_gap',
                       'safety_record', 'safety_clear', 'solid', 'cache', 'error_recoverable',
                       'error_black', 'error_exhausted', 'black', 'resume']
        if self.debug:
            self.phases += ['preview_finite', 'preview_expired', 'preview_white', 'preview_amber']
            capability = self.get(image.symbol('hardware')[0], 'led_hardware_info', 'capability')
            if capability not in (image.enum('LED_CAP_RGB_PWM'), image.enum('LED_CAP_RGB_PIXEL')):
                self.phases = [phase for phase in self.phases if phase not in ('preview_white', 'preview_amber')]
        self.phases += ['pre_quiesce', 'quiesce']

    def word(self, address):
        return int.from_bytes(self.cpu.mem_read(address, 4), 'little')

    def put_word(self, address, value):
        self.cpu.mem_write(address, (value & 0xFFFFFFFF).to_bytes(4, 'little'))

    def get(self, address, kind, name):
        offset, size, bits, shift = self.image.field(kind, name)
        value = int.from_bytes(self.cpu.mem_read(address + offset, size), 'little')
        return (value >> shift) & ((1 << bits) - 1)

    def put(self, address, kind, name, value):
        offset, size, bits, shift = self.image.field(kind, name)
        old = int.from_bytes(self.cpu.mem_read(address + offset, size), 'little')
        mask = ((1 << bits) - 1) << shift
        self.cpu.mem_write(address + offset,
                           ((old & ~mask) | ((value << shift) & mask)).to_bytes(size, 'little'))

    def member(self, address, kind, name):
        return address + self.image.field(kind, name)[0]

    def behavior(self, semantic):
        stride = self.image.structs['led_behavior'][0]
        value = self.image.enum(semantic)
        if not 0 < value < self.semantic_behaviors_size:
            raise ValueError(f'No linked behavior mapping for {semantic}')
        index = self.cpu.mem_read(self.semantic_behaviors + value, 1)[0]
        offset = index * stride
        if offset + stride > self.behaviors_size:
            raise ValueError(f'Invalid linked descriptor index for {semantic}: {index}')
        return self.behaviors + offset

    def reg(self, n):
        return self.cpu.reg_read(REGS[n])

    def ret(self, value=0):
        self.cpu.reg_write(UC_ARM_REG_R0, value & 0xFFFFFFFF)
        self.cpu.reg_write(UC_ARM_REG_R1, (value >> 32) & 0xFFFFFFFF if value >= 0 else 0)
        self.cpu.reg_write(UC_ARM_REG_PC, self.cpu.reg_read(UC_ARM_REG_LR))

    def call(self, address, *args):
        self.cpu.reg_write(UC_ARM_REG_SP, SETUP_TOP)
        self.cpu.reg_write(UC_ARM_REG_LR, SENTINEL | 1)
        for register, value in zip(REGS, args):
            self.cpu.reg_write(register, value)
        self.cpu.emu_start(address, SENTINEL, count=LIMIT)
        if self.cpu.reg_read(UC_ARM_REG_PC) != SENTINEL:
            raise RuntimeError('Driver initialization exceeded instruction limit')
        if self.reg(0) & 0x80000000:
            raise RuntimeError(f'Driver initialization failed: {self.reg(0):#x}')

    def initialize(self):
        devices = [(name, address) for name, (address, size) in self.image.symbols.items()
                   if name.startswith('__device_dts_ord_')
                   and size == self.image.structs['device'][0]]
        if self.kind == 'pixel':
            self.call(self.image.symbol('init_mem_slab_obj_core_list')[0])
        # Boot-ready state is a fixture, not device_is_ready() interception. Run
        # actual init for GPIO/PWM/I2S/pixel devices on a separate setup stack.
        for _, dev in devices:
            state = self.get(dev, 'device', 'state')
            self.put(state, 'device_state', 'init_res', 0)
            self.put(state, 'device_state', 'initialized', 1)
        selected = []
        for _, dev in devices:
            init = self.word(self.member(dev, 'device', 'ops'))
            name = self.image.functions.get(init & ~1, '')
            if name.startswith(('gpio_nrfx_init', 'pwm_nrfx_init', 'i2s_nrfx_init', 'ws2812_i2s_init')):
                selected.append((name, dev, init))
        for name, dev, init in sorted(selected, key=lambda item: (
                3 if item[0].startswith('ws2812') else 2 if item[0].startswith('i2s')
                else 1 if item[0].startswith('pwm') else 0)):
            data = self.get(dev, 'device', 'data')
            if name.startswith('pwm'):
                instance = self.member(data, 'pwm_nrfx_data', 'pwm')
                self.pwm_regs[self.word(instance)] = instance
            if name.startswith('i2s'):
                self.i2s_device, self.i2s_data = dev, data
            if name.startswith('ws2812'):
                self.pixel = dev
            self.call(init, dev)
            if name.startswith('i2s'):
                self.clock = self.get(data, 'i2s_nrfx_drv_data', 'clk_mgr')
                if not self.clock:
                    raise RuntimeError('I2S initialization did not provide a clock manager')
                self.put(self.clock, 'onoff_manager', 'flags', 2)
                self.put(self.clock, 'onoff_manager', 'refs', 1)
        if self.kind == 'pixel' and not (self.pixel and self.i2s_device):
            raise RuntimeError('Required pixel/I2S boot init was not found')
        if self.kind == 'pwm' and not self.pwm_regs:
            raise RuntimeError('Required PWM boot init was not found')
        self.setup_visits = dict(self.visits)
        self.visits.clear()
        self.trace.clear()

    def complete_dma(self):
        if self.i2s_data is None:
            return
        # Model hardware IRQ completion at the elapsed transfer wait, without
        # charging asynchronous data_handler stack to the worker. Ownership is
        # checked so a missing queue/slab implementation cannot fake success.
        for buffer in tuple(self.inflight):
            owners = [(slab, allocated) for slab, allocated in self.slabs.items() if buffer in allocated]
            if len(owners) != 1:
                raise RuntimeError('DMA completed a buffer without unique slab ownership')
            slab, allocated = owners[0]
            allocated.remove(buffer)
            # The asynchronous IRQ's free is excluded from the worker stack.
            # Preserve its nonblocking freelist/usage effect for real allocs.
            self.put_word(buffer, self.get(slab, 'k_mem_slab', 'free_list'))
            self.put(slab, 'k_mem_slab', 'free_list', buffer)
            info = self.member(slab, 'k_mem_slab', 'info')
            self.put(info, 'k_mem_slab_info', 'num_used',
                     self.get(info, 'k_mem_slab_info', 'num_used') - 1)
        self.inflight.clear()
        self.put(self.i2s_data, 'i2s_nrfx_drv_data', 'last_tx_buffer', 0)
        self.put(self.i2s_data, 'i2s_nrfx_drv_data', 'state', self.image.enum('I2S_STATE_READY'))
        instance = self.member(self.i2s_data, 'i2s_nrfx_drv_data', 'i2s')
        cb = self.member(instance, 'nrfx_i2s_t', 'cb')
        self.put(cb, 'nrfx_i2s_control_block_t', 'state', self.image.enum('NRFX_DRV_STATE_UNINITIALIZED'))
        self.put(self.clock, 'onoff_manager', 'refs', 1)

    def inject_phase(self):
        phase = self.phases[self.phase]
        delta = {'event_gap1': 200, 'event_second': 200, 'event_gap2': 200, 'event_third': 200,
                 'event_gap3': 200, 'event_fourth': 200, 'event_tail': 200,
                 'event_tail_end': 599, 'event_finish': 1, 'charging': 350,
                 'charging_fade': 650,  # Shared up-ramp at 1 s: distinct even with a 10% pixel cap.
                 'charging_same_slot': 0,
                 'stage_gap': 200, 'stage_tail': 599, 'stage_finish': 1,
                 'refusal_gap': 200, 'refusal_second': 200, 'refusal_tail': 200, 'refusal_finish': 600,
                 'cancel_gap': 200, 'cancel_finish': 600,
                 'button_plateau': 100, 'button_marker': 400, 'button_fade': 250,
                 'button_down': 500, 'button_black': 500, 'button_blink': 250, 'button_blink_gap': 500,
                 'fault_gap': 800, 'fault_second': 200, 'fault_pause': 800,
                 'fault_repeat': 3200, 'fault_repeat_gap': 800,
                 'safety_clear': 6000}.get(phase, 100)
        self.now = 10_000 if phase == 'ready' else self.now + delta
        if phase == 'charging_down':
            self.now = 12_500  # Interior of the common-clock CHARGE down-ramp.
        self.phase_visits[phase] += 1
        if phase in ('startup', 'cache', 'charging_fade', 'charging_same_slot',
                     'event_gap1', 'event_second', 'event_gap2', 'event_third',
                     'event_gap3', 'event_fourth', 'event_tail', 'event_tail_end', 'event_finish',
                     'stage_gap', 'stage_tail', 'stage_finish',
                     'refusal_gap', 'refusal_second', 'refusal_tail', 'refusal_finish',
                     'cancel_gap', 'cancel_finish', 'button_plateau', 'button_marker', 'button_fade',
                     'button_down', 'button_black', 'button_blink', 'button_blink_gap',
                     'fault_gap', 'fault_second', 'fault_pause', 'fault_repeat', 'fault_repeat_gap',
                     'safety_clear'):
            return
        # Emulate producer facts, never the worker's selection/envelope/output.
        # Reset policy only between independent phase fixtures; the hardware
        # state remains, so cache, black, resume and failure recovery are real.
        self.cpu.mem_write(self.engine, bytes(self.image.structs['led_engine'][0]))
        self.put(self.engine, 'led_engine', 'black_known', 1)
        self.put(self.engine, 'led_engine', 'black_since_ms', self.now - 600)
        self.put(self.engine, 'led_engine', 'identity_counter', self.phase * 10)
        connection = self.member(self.engine, 'led_engine', 'connection')
        for key in ('healthy', 'output_ready', 'radio_required', 'paired'):
            self.put(connection, 'led_connection_facts', key, 1)
        self.put(connection, 'led_connection_facts', 'radio_required', int(phase == 'ready'))
        if phase == 'ready':
            self.put(self.engine, 'led_engine', 'ready', 1)
        elif phase in ('charging', 'charging_down'):
            self.put(self.engine, 'led_engine', 'power', self.image.enum('LED_POWER_CHARGING'))
            self.put(connection, 'led_connection_facts', 'output_ready', 0)
            self.put(self.engine, 'led_engine', 'visible_origin_ms', self.now - 1100)
        elif phase in ('event', 'cancel', 'stage', 'refusal'):
            semantic = {'cancel': 'LED_CANCELLED', 'stage': 'LED_STAGE_ACK',
                        'refusal': 'LED_REJECTED'}.get(phase, 'LED_SUCCESS')
            event = self.member(self.engine, 'led_engine', 'events')
            for key, value in dict(present=1, request_id=1, event_id=1,
                                   occurred_ms=self.now,
                                   expires_ms=self.now + self.get(self.behavior(semantic), 'led_behavior', 'ttl_ms'),
                                   identity=self.phase * 10,
                                   semantic=self.image.enum(semantic)).items():
                self.put(event, 'led_pending_event', key, value)
        elif phase == 'button_hold':
            self.put(self.engine, 'led_engine', 'button_generation', 1)
            self.put(self.engine, 'led_engine', 'button_hold_active', 1)
            self.put(self.engine, 'led_engine', 'button_hold_origin_ms', self.now - 500)
            self.put(self.engine, 'led_engine', 'button_hold_identity', self.phase * 10)
        elif phase == 'fault':
            owner = self.member(self.engine, 'led_engine', 'owners')
            owner += self.image.enum('LED_OWNER_SENSOR') * self.image.structs['led_owner_state'][0]
            self.put(owner, 'led_owner_state', 'fault', self.image.enum('LED_FAULT_SENSOR'))
        elif phase == 'safety_record':
            owner = self.member(self.engine, 'led_engine', 'owners')
            owner += self.image.enum('LED_OWNER_TCAL') * self.image.structs['led_owner_state'][0]
            self.put(owner, 'led_owner_state', 'protection_id', 7)
            self.put(owner, 'led_owner_state', 'protection_record', 1)
            self.put(owner, 'led_owner_state', 'protection_expires_ms', self.now + 6000)
        elif phase.startswith('preview'):
            preview = self.member(self.engine, 'led_engine', 'preview')
            color = self.image.enum('LED_PHYSICAL_WHITE') if phase.endswith('white') else (
                self.image.enum('LED_PHYSICAL_AMBER') if phase.endswith('amber') else -1)
            for key, value in dict(generation=self.phase, console_session=1,
                                   semantic=self.image.enum('LED_SUCCESS'), color_override=color,
                                   origin_ms=self.now - 200,
                                   expires_ms=self.now - 1 if phase.endswith('expired') else self.now + 2000,
                                   status=self.image.enum('LED_PREVIEW_ACTIVE')).items():
                self.put(preview, 'led_preview_record', key, value)
            self.put(self.engine, 'led_engine', 'console_session', 1)
        elif phase in ('black', 'error_black', 'quiesce'):
            self.put(self.engine, 'led_engine', 'quiesced', int(phase == 'quiesce'))
        else:
            owner = self.member(self.engine, 'led_engine', 'owners')
            if phase == 'manual_exit':
                owner += self.image.enum('LED_OWNER_SYSTEM') * self.image.structs['led_owner_state'][0]
            for key, value in dict(session=1, request_id=1, revision=1,
                                   # Exhaustion follows an actual verified black
                                   # frame, so its peak request cannot be cache-hit.
                                   origin_ms=self.now - (0 if phase == 'error_exhausted' else 350),
                                   identity=self.phase * 10,
                                   semantic=self.image.enum(
                                       'LED_MANUAL_EXIT' if phase == 'manual_exit' else
                                       'LED_EXIT_PENDING' if phase.startswith('error') else 'LED_WAIT_STILL')).items():
                self.put(owner, 'led_owner_state', key, value)

    def mmio(self, cpu, access, address, size, value, user_data):
        # nRF52 PWM TASKS_STOP=0x004, EVENTS_STOPPED=0x104. Keep actual
        # stop/stopped-check/playback code, replacing only peripheral response.
        for base in self.pwm_regs:
            if address == base + 4 and value:
                self.put_word(base + 0x104, 1)
                # With a registered callback nrfx's stopped-check waits for the
                # asynchronous IRQ to update its control block. Model that
                # completed IRQ state, not a fake successful driver return.
                cb = self.member(self.pwm_regs[base], 'nrfx_pwm_t', 'cb')
                self.put(cb, 'nrfx_pwm_control_block_t', 'state',
                         self.image.enum('NRFX_DRV_STATE_INITIALIZED'))
                self.stubs['PWM_STOPPED_MMIO_response'] += 1
        if self.i2s_data is not None:
            instance = self.member(self.i2s_data, 'i2s_nrfx_drv_data', 'i2s')
            base = self.word(instance)
            if address == base + self.image.field('NRF_I2S_Type', 'TXD')[0] and value:
                if not any(value in allocated for allocated in self.slabs.values()):
                    raise RuntimeError('I2S DMA pointer is not slab-owned')
                self.inflight.add(value)
                self.visits['I2S_DMA_TX_PTR'] += 1

    def hook(self, cpu, pc, size, user_data):
        self.instructions += 1
        opcode = bytes(cpu.mem_read(pc, size))
        first = int.from_bytes(opcode[:2], 'little')
        call = ((size == 4 and first & 0xF800 == 0xF000
                 and int.from_bytes(opcode[2:], 'little') & 0xD000 == 0xD000)
                or (size == 2 and first & 0xFF87 == 0x4780))
        if call:
            # Unicorn leaked expired ITGE (xPSR0x8100a800) across BL@0xa752,
            # skipping the callee PUSH. An executed BL/BLX consumes its last
            # IT slot; no caller IT scope enters the callee. Preserve NZCV/T.
            xpsr = cpu.reg_read(UC_ARM_REG_XPSR)
            if xpsr & 0x0600FC00:
                cpu.reg_write(UC_ARM_REG_XPSR, xpsr & ~0x0600FC00)
                self.stubs['ARM_call_ITSTATE_boundary'] += 1
        if self.pending_alloc and pc == self.pending_alloc[-1][0]:
            _, slab, output = self.pending_alloc.pop()
            if self.reg(0) == 0:
                buffer = self.word(output)
                allocated = self.slabs.setdefault(slab, set())
                if buffer in allocated:
                    raise RuntimeError('Actual slab allocation returned an owned block')
                allocated.add(buffer)
        if self.measuring:
            sp = cpu.reg_read(UC_ARM_REG_SP)
            phase = self.phases[self.phase]
            self.phase_lowest[phase] = min(self.phase_lowest.get(phase, self.top), sp)
            if sp < self.lowest:
                self.lowest, self.deepest = sp, list(self.trace)[-12:]
            for start, end, inline_name in self.image.inline_ranges:
                key = (inline_name, self.phase)
                if start <= pc < end and key not in self.inline_seen:
                    self.inline_visits[inline_name] += 1
                    self.inline_seen.add(key)
        name = self.image.functions.get(pc)
        if name is None:
            return
        self.visits[name] += 1
        self.trace.append(name)
        phase = self.phases[self.phase]
        if 'assert' in name or 'fatal' in name:
            raise RuntimeError(f'Unexpected firmware failure: {name}')
        if name.startswith('k_uptime_get'):
            self.stubs[name] += 1
            self.ret(self.now)
        elif name in ('sys_clock_tick_get', 'k_uptime_ticks'):
            self.stubs[name] += 1
            self.ret(self.now * int(self.image.config['CONFIG_SYS_CLOCK_TICKS_PER_SEC']) // 1000)
        elif name in ('z_impl_k_sem_take', 'k_sem_take'):
            self.stubs[name] += 1
            if self.reg(0) != self.image.symbol('led_changed')[0]:
                raise RuntimeError('Unexpected worker semaphore wait')
            winner = self.member(self.engine, 'led_engine', 'winner')
            selected = self.get(winner, 'led_selection', 'semantic')
            self.semantics.add(selected)
            self.phase_semantics[phase] = selected
            self.phase_errors[phase] = self.word(self.image.symbol('reported_error')[0])
            self.phase_powered[phase] = self.get(self.image.symbol('hardware')[0], 'led_hardware_info', 'powered')
            if self.debug:
                preview = self.member(self.engine, 'led_engine', 'preview')
                self.phase_previews[phase] = self.get(preview, 'led_preview_record', 'status')
            if self.phase == len(self.phases) - 1:
                self.completed = True
                cpu.emu_stop()
            else:
                self.phase += 1
                self.inject_phase()
                self.ret()
        elif name in ('z_impl_k_sleep', 'z_impl_k_usleep', 'k_usleep', 'k_msleep'):
            self.stubs[name] += 1
            if self.kind == 'pixel':
                self.complete_dma()
            self.ret()
        elif name in ('k_sched_lock', 'k_sched_unlock', 'z_impl_k_sem_give',
                      'k_sem_give', 'z_impl_k_mutex_lock', 'z_impl_k_mutex_unlock',
                      'k_mutex_init', 'z_impl_k_mutex_init', 'irq_enable',
                      'arch_irq_enable', 'z_arm_irq_priority_set'):
            self.stubs[name] += 1
            self.ret()
        elif name == 'k_mem_slab_alloc':
            self.pending_alloc.append((cpu.reg_read(UC_ARM_REG_LR) & ~1, self.reg(0), self.reg(1)))
        elif name == 'k_mem_slab_free':
            allocated = self.slabs.setdefault(self.reg(0), set())
            if self.reg(1) not in allocated:
                raise RuntimeError(f'Double/unowned slab free: {self.reg(1):#x}')
            allocated.remove(self.reg(1))
            self.inflight.discard(self.reg(1))
        elif name.startswith('z_log_msg_finalize'):
            self.stubs[name] += 1
            self.finalized += int(self.measuring)
            self.ret()
        elif name == 'z_log_msg_alloc':
            self.stubs[name] += 1
            self.packages += int(self.measuring)
            length = self.reg(0) * 4
            if not length or self.log_cursor + length > SCRATCH + 0xC0000:
                raise RuntimeError('Invalid/excessive log allocation')
            result = self.log_cursor
            self.log_cursor += (length + 7) & ~7
            self.ret(result)
        elif name == 'cbprintf_package_convert':
            self.conversions += int(self.measuring)
        elif name == 'pwm_nrfx_set_cycles' and self.measuring:
            self.outputs += 1
            self.phase_outputs[phase] += 1
            self.pwm_channels.add(self.reg(1))
            self.nonzero_outputs += bool(self.reg(3))
            self.phase_nonzero_outputs[phase] += bool(self.reg(3))
            if phase in ('error_recoverable', 'error_exhausted') and (phase == 'error_exhausted' or not self.injected):
                self.injected += 1
                self.phase_injected[phase] += 1
                cpu.reg_write(UC_ARM_REG_R1, 4)  # actual driver's invalid-channel error/log
        elif name == 'ws2812_strip_update_rgb' and self.measuring:
            self.outputs += 1
            self.phase_outputs[phase] += 1
            rgb = bytes(cpu.mem_read(self.reg(1), self.image.structs['led_rgb'][0]))
            self.nonzero_outputs += any(rgb)
            self.phase_nonzero_outputs[phase] += any(rgb)
            self.phase_pixel_frames.setdefault(phase, []).append(
                [self.get(self.reg(1), 'led_rgb', channel) for channel in ('r', 'g', 'b')])
        elif name == 'nrfx_i2s_start':
            buffers = self.reg(1)
            tx = self.get(buffers, 'nrfx_i2s_buffers_t', 'p_tx_buffer')
            if not tx or not any(tx in allocated for allocated in self.slabs.values()):
                raise RuntimeError('I2S START lacks a slab-owned queued TX block')
            self.inflight.add(tx)
        elif name == 'i2s_nrfx_trigger' and self.measuring:
            instance = self.member(self.i2s_data, 'i2s_nrfx_drv_data', 'i2s')
            cb = self.member(instance, 'nrfx_i2s_t', 'cb')
            command = self.reg(2)
            self.i2s_commands[command] += 1
            if command == self.image.enum('I2S_TRIGGER_START') and phase in ('error_recoverable', 'error_exhausted') and (
                    phase == 'error_exhausted' or not self.injected):
                self.injected += 1
                self.phase_injected[phase] += 1
                self.put(cb, 'nrfx_i2s_control_block_t', 'state',
                         self.image.enum('NRFX_DRV_STATE_INITIALIZED'))
            elif command in (self.image.enum('I2S_TRIGGER_PREPARE'), self.image.enum('I2S_TRIGGER_DROP')):
                # Restore the externally injected peripheral condition. The
                # actual trigger/recovery queue purge still owns/free buffers.
                self.put(cb, 'nrfx_i2s_control_block_t', 'state',
                         self.image.enum('NRFX_DRV_STATE_UNINITIALIZED'))
        elif name == 'pm_device_action_run' and self.measuring:
            self.pm_actions[self.reg(1)] += 1

    def run(self):
        failures = []
        try:
            self.initialize()
            self.phase_visits['startup'] += 1
            self.cpu.reg_write(UC_ARM_REG_SP, self.top)
            self.cpu.reg_write(UC_ARM_REG_LR, SENTINEL | 1)
            self.measuring = True
            self.cpu.emu_start(self.entry, SENTINEL, count=LIMIT)
        except (UcError, RuntimeError, ValueError, KeyError) as exc:
            self.error = f'{type(exc).__name__}: {exc}; pc={self.cpu.reg_read(UC_ARM_REG_PC):#x}; recent={list(self.trace)}'
            failures.append(self.error)
        used = self.top - self.lowest
        if not self.completed:
            failures.append('Required worker scenario did not complete')
        if used > self.budget:
            failures.append(f'Measured {used} bytes exceeds linked usable budget {self.budget}')
        if self.completed:
            if not self.nonzero_outputs or not self.outputs:
                failures.append('No actual nonzero driver output visited')
            if not self.injected or not self.finalized or not self.packages:
                failures.append('Required driver error and actual cbprintf log packaging/finalize not visited')
            if self.phase_outputs['cache']:
                failures.append('Unchanged-frame cache fixture unexpectedly called the driver')
            if self.phase_outputs['button_plateau'] or self.phase_outputs['charging_same_slot']:
                failures.append('Steady ON or same-slot wake rewrote a successfully latched frame')
            for phase in ('ready', 'charging_fade', 'charging_down', 'event', 'event_tail', 'black', 'resume', 'quiesce',
                          'event_gap1', 'event_second', 'event_gap2', 'event_third', 'event_gap3', 'event_fourth',
                          'button_hold', 'button_marker', 'button_fade', 'button_down', 'button_black',
                          'button_blink', 'button_blink_gap', 'manual_exit', 'cancel', 'cancel_gap',
                          'stage', 'stage_gap', 'refusal', 'refusal_gap', 'refusal_second', 'refusal_tail',
                          'fault', 'fault_gap', 'fault_second', 'fault_pause', 'fault_repeat', 'fault_repeat_gap',
                          'safety_record', 'safety_clear', 'error_recoverable', 'error_black', 'error_exhausted'):
                if not self.phase_outputs[phase]:
                    failures.append(f'Required driver-output phase was not visited: {phase}')
            for phase in ('charging_fade', 'charging_down',
                          'button_hold', 'button_fade', 'button_down', 'button_blink', 'manual_exit',
                          'event', 'event_second', 'event_third', 'event_fourth',
                          'stage', 'refusal', 'refusal_second', 'cancel',
                          'fault', 'fault_second', 'fault_repeat', 'safety_record'):
                if not self.phase_nonzero_outputs[phase]:
                    failures.append(f'Glyph ON phase did not emit actual light: {phase}')
            for phase in ('event_gap1', 'event_gap2', 'event_gap3', 'event_tail', 'event_tail_end',
                          'stage_gap', 'stage_tail', 'refusal_gap', 'refusal_tail',
                          'cancel_gap', 'button_marker', 'button_black', 'button_blink_gap',
                          'fault_gap', 'fault_pause', 'fault_repeat_gap'):
                if self.phase_nonzero_outputs[phase]:
                    failures.append(f'Glyph dark phase emitted actual light: {phase}')
            expected_semantics = {
                'ready': 'LED_READY', 'event': 'LED_SUCCESS', 'event_tail': 'LED_SUCCESS',
                'event_gap1': 'LED_SUCCESS', 'event_second': 'LED_SUCCESS',
                'event_gap2': 'LED_SUCCESS', 'event_third': 'LED_SUCCESS',
                'event_gap3': 'LED_SUCCESS', 'event_fourth': 'LED_SUCCESS', 'event_tail_end': 'LED_SUCCESS',
                'stage': 'LED_STAGE_ACK', 'stage_gap': 'LED_STAGE_ACK', 'stage_tail': 'LED_STAGE_ACK',
                'refusal': 'LED_REJECTED', 'refusal_gap': 'LED_REJECTED',
                'refusal_second': 'LED_REJECTED', 'refusal_tail': 'LED_REJECTED',
                'cancel': 'LED_CANCELLED', 'cancel_gap': 'LED_CANCELLED',
                'fault': 'LED_SENSOR_FAULT', 'fault_gap': 'LED_SENSOR_FAULT',
                'fault_second': 'LED_SENSOR_FAULT', 'fault_pause': 'LED_SENSOR_FAULT',
                'fault_repeat': 'LED_SENSOR_FAULT', 'fault_repeat_gap': 'LED_SENSOR_FAULT',
                'safety_record': 'LED_SAFETY_FAULT',
                'button_hold': 'LED_BUTTON_HOLD', 'button_plateau': 'LED_BUTTON_HOLD',
                'button_marker': 'LED_BUTTON_HOLD', 'button_fade': 'LED_BUTTON_HOLD',
                'button_down': 'LED_BUTTON_HOLD', 'button_black': 'LED_BUTTON_HOLD',
                'button_blink': 'LED_BUTTON_HOLD', 'button_blink_gap': 'LED_BUTTON_HOLD',
                'manual_exit': 'LED_MANUAL_EXIT'
            }
            for phase, semantic in expected_semantics.items():
                if self.phase_semantics.get(phase) != self.image.enum(semantic):
                    failures.append(f'Wrong actual selected semantic in {phase}: expected {semantic}')
            for phase, semantic in (('event_finish', 'LED_SUCCESS'), ('stage_finish', 'LED_STAGE_ACK'),
                                    ('refusal_finish', 'LED_REJECTED'), ('cancel_finish', 'LED_CANCELLED')):
                if self.phase_semantics.get(phase) == self.image.enum(semantic):
                    failures.append(f'Finite receipt did not retire at its complete envelope: {phase}')
            if self.phase_nonzero_outputs['error_black']:
                failures.append('Error bridge did not emit an actual black driver frame')
            if self.phase_powered.get('error_black') != 0:
                failures.append('Error bridge did not cut off the hardware rail')
            if not self.phase_nonzero_outputs['error_exhausted']:
                failures.append('Exhausted driver fault did not receive a nonzero frame')
            for phase in ('error_recoverable', 'error_exhausted'):
                if not self.phase_injected[phase]:
                    failures.append(f'Required actual driver fault was not injected: {phase}')
            if self.kind == 'pixel' and self.phase_errors.get('error_recoverable') != 0:
                failures.append('Recoverable START failure did not recover on its real retry')
            if not self.phase_errors.get('error_exhausted'):
                failures.append('Exhausted driver failure was incorrectly reported as success')
            for phase in ('error_black', 'black', 'resume', 'quiesce'):
                if self.phase_errors.get(phase):
                    failures.append(f'Hardware did not recover for {phase}')
            if self.debug:
                if self.phase_previews.get('preview_expired') != self.image.enum('LED_PREVIEW_EXPIRED'):
                    failures.append('Actual preview deadline expiry was not visited')
                for phase in ('preview_finite', 'preview_white', 'preview_amber'):
                    if phase in self.phases and not self.phase_outputs[phase]:
                        failures.append(f'Required debug-preview output was not visited: {phase}')
            required = ['pm_device_action_run', 'pwm_nrfx_set_cycles'] if self.kind == 'pwm' else [
                'ws2812_strip_update_rgb', 'ws2812_strip_update', 'i2s_nrfx_write',
                'i2s_nrfx_trigger', 'start_transfer', 'nrfx_i2s_start', 'onoff_request']
            for name in required:
                if not (self.visits[name] or self.inline_visits[name]):
                    failures.append(f'Required linked driver chain was not visited: {name}')
            if self.kind == 'pwm' and not self.visits['nrfx_pwm_stopped_check']:
                failures.append('Actual PWM stop/suspend/resume path was not visited')
            if self.kind == 'pwm' and not (
                    self.visits['pwm_nrfx_pm_action'] and self.visits['pinctrl_apply_state']
                    and (self.visits['nrfx_pwm_simple_playback'] or self.inline_visits['nrfx_pwm_simple_playback'])):
                failures.append('Actual PWM playback/PM/pinctrl chain was not visited')
            if self.kind == 'pixel' and any(self.slabs.values()):
                failures.append('Pixel scenario leaked slab ownership')
            if self.kind == 'pixel':
                for command in ('I2S_TRIGGER_START', 'I2S_TRIGGER_DRAIN',
                                'I2S_TRIGGER_PREPARE', 'I2S_TRIGGER_DROP'):
                    if not self.i2s_commands[self.image.enum(command)]:
                        failures.append(f'Required actual I2S branch was not visited: {command}')
                if self.pending_alloc:
                    failures.append('Actual slab allocation did not return')
        return dict(case=self.case, hardware_kind=self.kind, debug=self.debug,
                    linked_stack_budget=self.budget, stack_storage_bytes=self.storage,
                    measured_thread_stack=used, lowest_sp=hex(self.lowest),
                    headroom_excluding_entry_interrupts_and_stubbed_leaves=self.budget - used,
                    completed=self.completed, driver_outputs=self.outputs,
                    nonzero_driver_outputs=self.nonzero_outputs, injected_errors=self.injected,
                    allocated_log_packages=self.packages,
                    cbprintf_conversion_calls=self.conversions, finalized_logs=self.finalized,
                    pwm_channels=sorted(self.pwm_channels), pm_actions=dict(self.pm_actions),
                    i2s_trigger_commands=dict(self.i2s_commands),
                    phase_outputs=dict(self.phase_outputs), phases_visited=dict(self.phase_visits),
                    phase_injected_errors=dict(self.phase_injected),
                    phase_pixel_frames=self.phase_pixel_frames,
                    phase_nonzero_driver_outputs=dict(self.phase_nonzero_outputs),
                    phase_selected_semantics=self.phase_semantics,
                    phase_stack_bytes={phase: self.top - low for phase, low in self.phase_lowest.items()},
                    phase_hardware_errors=self.phase_errors, phase_preview_status=self.phase_previews,
                    phase_hardware_powered=self.phase_powered,
                    descriptor_styles={name: self.get(self.behavior(name), 'led_behavior', 'style')
                                       for name in ('LED_STAGE_ACK', 'LED_SUCCESS', 'LED_CANCELLED', 'LED_REJECTED',
                                                    'LED_SENSOR_MISSING', 'LED_SENSOR_FAULT', 'LED_SAFETY_FAULT')},
                    selected_semantics=sorted(self.semantics), actual_function_visits=dict(self.visits),
                    actual_inlined_driver_visits=dict(self.inline_visits),
                    replaced_boundary_calls=dict(self.stubs),
                    setup_function_visits=getattr(self, 'setup_visits', {}),
                    instructions=self.instructions, deepest_recent_functions=self.deepest,
                    failures=failures)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('elf', type=Path, nargs='+', help='Linked tracker application ELF(s) with adjacent .config')
    args = parser.parse_args()
    reports = []
    for path in args.elf:
        try:
            image = Image(path)
            report = dict(elf=str(path), sha256=image.sha256,
                          board=image.config.get('CONFIG_BOARD_TARGET'),
                          config={name: image.config.get(name, 'n') for name in (
                              'CONFIG_LTO', 'CONFIG_LED_DEBUG', 'CONFIG_LED_THREAD_STACK_SIZE',
                              'CONFIG_STACK_USAGE', 'CONFIG_PM_DEVICE', 'CONFIG_HW_STACK_PROTECTION')},
                          result=Replay(image, 'normal-errors-black-resume-preview-quiesce').run())
        except (OSError, ValueError, KeyError) as exc:
            report = dict(elf=str(path), result=dict(failures=[f'{type(exc).__name__}: {exc}']))
        reports.append(report)
    print(json.dumps(dict(measurement='lower_bound_not_hardware_proof', images=reports), indent=2))
    return int(any(report['result']['failures'] for report in reports))


if __name__ == '__main__':
    raise SystemExit(main())
