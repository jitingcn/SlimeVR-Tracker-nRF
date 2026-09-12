#!/usr/bin/env python3
"""Compile actual ICM426xx init/FIFO/temp lifecycle against a register/FIFO model."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile
HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get('SOURCE_ROOT', HERE.parents[2]))
# Baseline trees may contain only saved changed files; unchanged headers come from HEADER_ROOT.
HEADERS = Path(os.environ.get('HEADER_ROOT', HERE.parents[2]))

def function(source, name):
    match = re.search(rf'^(?:static )?(?:void|int|float|uint16_t) {name}\([^;{{]*\)\s*\{{', source, re.MULTILINE)
    if match is None: raise ValueError(name)
    start = source.index('{', match.start())
    tokens = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[{}]', re.DOTALL)
    depth = 0
    for token in tokens.finditer(source, start):
        if token.group() == '{': depth += 1
        elif token.group() == '}':
            depth -= 1
            if depth == 0: return source[match.start():token.end()]
    raise ValueError(name)

preamble = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#define MHZ(x) ((x)*1000000)
#define SENSOR_INTERFACE_DEV_IMU 0
#define LOG_ERR(...) ((void)0)
#define LOG_WRN(...) ((void)0)
#define LOG_INF(...) ((void)0)
static uint8_t fifo_model[80];
static uint16_t available, next_chunk_available;
static unsigned model_offset, data_reads;
static bool fail_second_data;
static int count_error, data_error, reg_error, init_error;
static int16_t register_temp;
static unsigned register_reads;
static bool fail_second_count;
static unsigned count_reads;
static int sensor_interface_spi_configure(int dev, int hz, int mode) {(void)dev;(void)hz;(void)mode;return 0;}
static void k_msleep(int ms) {(void)ms;}
static int ssi_reg_write_byte(int dev, uint8_t reg, uint8_t value) {
    (void)dev;(void)reg;(void)value;return init_error;
}
static int ssi_reg_update_byte(int dev, uint8_t reg, uint8_t mask, uint8_t value) {
    (void)dev;(void)reg;(void)mask;(void)value; return init_error;
}
static int ssi_reg_read_byte(int dev, uint8_t reg, uint8_t *value) {(void)dev;(void)reg;*value=0;return 0;}
static int ssi_burst_read(int dev, uint8_t reg, uint8_t *out, uint16_t len) {
    (void)dev; assert(len == 2);
    if (reg == COUNT_REG) {
        count_reads++;
        if (count_error || (fail_second_count && count_reads == 2)) return -1;
        if (available == 0 && next_chunk_available != 0) {
            available=next_chunk_available;next_chunk_available=0;
        }
        out[0]=available>>8;out[1]=available;return 0;
    }
    assert(reg == TEMP_REG);
    register_reads++;
    if (reg_error) return -1;
    out[0]=(uint16_t)register_temp>>8;out[1]=register_temp;return 0;
}
static int ssi_burst_read_interval(int dev, uint8_t reg, uint8_t *out, uint16_t len, uint16_t stride) {
    (void)dev;(void)reg;assert(stride == 20);assert(model_offset+len <= sizeof(fifo_model));
    data_reads++;
    if (data_error || (fail_second_data && data_reads == 2)) return -1;
    memcpy(out,&fifo_model[model_offset],len);model_offset+=len;available=0;return 0;
}
static int UPDATE_ODR(float a,float g,float *out_a,float *out_g) {*out_a=a;*out_g=g;return 0;}
'''
main = r'''
static uint8_t buffer[100];
static void packet(unsigned index, uint8_t header, int16_t temperature) {
    uint8_t *p = &fifo_model[index*20];
    memset(p, 0, 20);p[0]=header;
    p[13]=(uint16_t)temperature>>8;p[14]=temperature;
}
static void load(uint16_t n) {
    available=n;count_reads=0;data_reads=0;model_offset=0;next_chunk_available=0;
}
static void close_to(float actual, float expected) {assert(isfinite(actual));assert(fabsf(actual-expected)<0.00001f);}
static void cache_one(int16_t raw) {
    packet(0,0x78,raw);load(1);
    assert(FIFO_READ(buffer,sizeof(buffer)) == 1);
}
int main(void) {
    float a,g;
    assert(INIT(0,0.0025f,0.0025f,&a,&g) == 0);
    register_temp=1325;
    close_to(TEMP_READ(),(float)1325/132.48f+25);
    packet(0,0x78,1325);packet(1,0x7A,2650);packet(2,0x80,0);load(3);
    unsigned reads_before=register_reads;
    assert(FIFO_READ(buffer,sizeof(buffer)) == 3);
    close_to(TEMP_READ(),(float)2650/132.48f+25); /* newest real sample, not mean or empty read */
    assert(register_reads == reads_before);
    /* A new sample arriving during the drain supersedes the first burst. */
    packet(0,0x78,2000);packet(1,0x78,3000);load(1);next_chunk_available=1;
    reads_before=register_reads;
    assert(FIFO_READ(buffer,sizeof(buffer)) == 2);
    close_to(TEMP_READ(),(float)3000/132.48f+25);
    assert(register_reads == reads_before);
    /* Valid high-res negative temperature and malformed/invalid trailing records. */
    packet(0,0x78,-5300);packet(1,0x78,INT16_MIN);packet(2,0x68,4000);packet(3,0x7F,5000);load(4);
    assert(FIFO_READ(buffer,sizeof(buffer)) == 4);
    close_to(TEMP_READ(),(float)-5300/132.48f+25);
    load(0);assert(FIFO_READ(buffer,sizeof(buffer)) == 0);
    close_to(TEMP_READ(),(float)register_temp/132.48f+25); /* empty acquisition cannot reuse prior batch */
    cache_one(2000);load(1);count_error=1;
    assert(FIFO_READ(buffer,sizeof(buffer)) == 0);count_error=0;reg_error=1;
    assert(isnan(TEMP_READ()));reg_error=0;
    cache_one(2000);load(1);data_error=1;
    assert(FIFO_READ(buffer,sizeof(buffer)) == 0);data_error=0;reg_error=1;
    assert(isnan(TEMP_READ()));reg_error=0;
    cache_one(2000);packet(0,0x78,3000);load(1);fail_second_count=true;
    assert(FIFO_READ(buffer,sizeof(buffer)) == 1);fail_second_count=false;reg_error=1;
    assert(isnan(TEMP_READ()));reg_error=0; /* partial FIFO followed by bus failure invalidates cache */
    packet(0,0x78,2000);packet(1,0x78,3000);load(1);next_chunk_available=1;fail_second_data=true;
    assert(FIFO_READ(buffer,sizeof(buffer)) == 1);fail_second_data=false;reg_error=1;
    assert(isnan(TEMP_READ()));reg_error=0;
    cache_one(2000);SHUTDOWN();reg_error=1;assert(isnan(TEMP_READ()));reg_error=0;
    cache_one(2000);init_error=-1;assert(INIT(0,0.0025f,0.0025f,&a,&g)<0);
    init_error=0;reg_error=1;assert(isnan(TEMP_READ()));reg_error=0;
    packet(0,0x78,INT16_MIN);load(1);assert(FIFO_READ(buffer,sizeof(buffer))==1);
    close_to(TEMP_READ(),(float)register_temp/132.48f+25);
    puts("FIFO temperature: full-resolution newest sample; empty, invalid, read failures and lifecycle invalidate");
    return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='sensor-temp-') as directory:
    tmp=Path(directory)
    for model,prefix in [('ICM42688','icm'),('ICM42686','icm42686')]:
        source=(ROOT/f'src/sensor/imu/{model}.c').read_text()
        header=(HEADERS/f'src/sensor/imu/{model}.h').read_text()
        defines='\n'.join(line for line in header.splitlines() if line.startswith('#define '))
        names={'INIT':f'{prefix}_init','SHUTDOWN':f'{prefix}_shutdown','FIFO_READ':f'{prefix}_fifo_read','TEMP_READ':f'{prefix}_temp_read','UPDATE_ODR':f'{prefix}_update_odr','COUNT_REG':f'{model}_FIFO_COUNTH','TEMP_REG':f'{model}_TEMP_DATA1'}
        aliases='\n'.join(f'#define {key} {value}' for key,value in names.items())
        state=source[source.index('static uint8_t last_accel_odr'):source.index('LOG_MODULE_REGISTER')]
        decoder=(ROOT/'src/sensor/imu/icm426xx_hires.h').read_text()
        unit=tmp/f'{model}.c'
        unit.write_text(defines+'\n'+aliases+'\n'+preamble+'\n'+decoder+'\n#define PACKET_SIZE ICM426XX_HIRES_PACKET_SIZE\n'+state+'\n'+'\n'.join(function(source,names[key]) for key in ('INIT','SHUTDOWN','FIFO_READ','TEMP_READ'))+main)
        binary=tmp/model
        subprocess.run(shlex.split(os.environ.get('CC','cc'))+['-std=c11','-Wall','-Wextra','-Werror','-Wno-unused-function','-g','-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer','-fno-pie','-no-pie',str(unit),'-lm','-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
