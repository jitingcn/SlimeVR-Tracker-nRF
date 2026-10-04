CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
FLAGS = $(CFLAGS) -I.
DEPS = test_led_hw.c globals.h Makefile ../../../src/system/led_hw.c ../../../src/system/led.h \
       ../../../src/system/led_internal.h ../../../src/system/led_strip_fade.h \
       ../../../src/system/led_timeline.c ../../../src/system/led_behavior.c \
       zephyr/kernel.h zephyr/drivers/gpio.h zephyr/drivers/pwm.h zephyr/drivers/led_strip.h zephyr/pm/device.h
BINS = test_noled test_gpio test_gpio_ungated test_gpio_dim test_gpio_muted test_mono_pwm test_mono_pwm_muted \
       test_dual_pwm test_dual_unknown test_dual_gpio test_rg_mix test_tri_pwm test_tri_gpio \
       test_rgb_pwm test_rgb_rbg_inverted test_rgb_limited test_pixel test_pixel_factory_dim \
       test_pixel_p10_default test_pixel_muted test_pixel_ungated test_dual_primary \
       test_rg_missing_red test_rg_missing_green test_rgb_missing_red test_rgb_missing_green test_rgb_missing_blue
FADE_BINS = test_pixel_fade_limited test_pixel_fade_weighted test_pixel_fade_rbg \
            test_pixel_fade_low_total test_pixel_fade_zero_cap test_pixel_fade_dim_cap
.PHONY: check clean
check: $(BINS) $(FADE_BINS)
	@set -e; for test in $(BINS); do ./$$test; done; for test in $(FADE_BINS); do ./$$test --fade-regression; done

define fixture
test_$(1): $(DEPS)
	$(CC) $(FLAGS) $(2) -o $$@ $$<
endef
$(eval $(call fixture,noled,-DHOST_KIND=0 -DHOST_KIND_TOKEN=no_led))
$(eval $(call fixture,gpio,-DHOST_KIND=1 -DHOST_KIND_TOKEN=mono_gpio))
$(eval $(call fixture,gpio_ungated,-DHOST_KIND=1 -DHOST_KIND_TOKEN=mono_gpio -DHOST_GATE=0 -DHOST_POLARITY=1))
$(eval $(call fixture,gpio_dim,-DHOST_KIND=1 -DHOST_KIND_TOKEN=mono_gpio -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=1))
$(eval $(call fixture,gpio_muted,-DHOST_KIND=1 -DHOST_KIND_TOKEN=mono_gpio -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=0))
$(eval $(call fixture,mono_pwm,-DHOST_KIND=2 -DHOST_KIND_TOKEN=mono_pwm))
$(eval $(call fixture,mono_pwm_muted,-DHOST_KIND=2 -DHOST_KIND_TOKEN=mono_pwm -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=0))
$(eval $(call fixture,dual_pwm,-DHOST_KIND=3 -DHOST_KIND_TOKEN=dual_discrete))
$(eval $(call fixture,dual_unknown,-DHOST_KIND=3 -DHOST_KIND_TOKEN=dual_discrete -DHOST_UNKNOWN_COLORS=1))
$(eval $(call fixture,dual_primary,-DHOST_KIND=3 -DHOST_KIND_TOKEN=dual_discrete -DHOST_UNKNOWN_COLORS=1 -DHOST_PRIMARY=1 -DHOST_BOARD_LIMIT=4000 -DHOST_TOTAL_LIMIT=7000))
$(eval $(call fixture,dual_gpio,-DHOST_KIND=3 -DHOST_KIND_TOKEN=dual_discrete -DHOST_GPIO_TRANSPORT=1))
$(eval $(call fixture,rg_mix,-DHOST_KIND=4 -DHOST_KIND_TOKEN=rg_mix))
$(eval $(call fixture,rg_missing_red,-DHOST_KIND=4 -DHOST_KIND_TOKEN=rg_mix -DHOST_MISSING_COLOR=1))
$(eval $(call fixture,rg_missing_green,-DHOST_KIND=4 -DHOST_KIND_TOKEN=rg_mix -DHOST_MISSING_COLOR=2))
$(eval $(call fixture,tri_pwm,-DHOST_KIND=5 -DHOST_KIND_TOKEN=tri_discrete))
$(eval $(call fixture,tri_gpio,-DHOST_KIND=5 -DHOST_KIND_TOKEN=tri_discrete -DHOST_GPIO_TRANSPORT=1))
$(eval $(call fixture,rgb_pwm,-DHOST_KIND=6 -DHOST_KIND_TOKEN=rgb_pwm))
$(eval $(call fixture,rgb_missing_red,-DHOST_KIND=6 -DHOST_KIND_TOKEN=rgb_pwm -DHOST_MISSING_COLOR=1))
$(eval $(call fixture,rgb_missing_green,-DHOST_KIND=6 -DHOST_KIND_TOKEN=rgb_pwm -DHOST_MISSING_COLOR=2))
$(eval $(call fixture,rgb_missing_blue,-DHOST_KIND=6 -DHOST_KIND_TOKEN=rgb_pwm -DHOST_MISSING_COLOR=3))
$(eval $(call fixture,rgb_rbg_inverted,-DHOST_KIND=6 -DHOST_KIND_TOKEN=rgb_pwm -DHOST_RBG=1 -DHOST_POLARITY=1))
$(eval $(call fixture,rgb_limited,-DHOST_KIND=6 -DHOST_KIND_TOKEN=rgb_pwm -DHOST_BOARD_LIMIT=4000 -DHOST_TOTAL_LIMIT=7000))
$(eval $(call fixture,pixel,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel))
$(eval $(call fixture,pixel_factory_dim,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=1000))
$(eval $(call fixture,pixel_p10_default,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=2000))
$(eval $(call fixture,pixel_muted,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=0))
$(eval $(call fixture,pixel_ungated,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DHOST_GATE=0))
$(eval $(call fixture,pixel_fade_limited,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DHOST_BOARD_LIMIT=4000 -DHOST_TOTAL_LIMIT=7000))
$(eval $(call fixture,pixel_fade_weighted,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DHOST_RED_WEIGHT=7000 -DHOST_GREEN_WEIGHT=9000 -DHOST_BLUE_WEIGHT=4000))
$(eval $(call fixture,pixel_fade_rbg,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DHOST_RBG=1 -DHOST_RED_WEIGHT=6000 -DHOST_GREEN_WEIGHT=3000 -DHOST_BLUE_WEIGHT=10000))
$(eval $(call fixture,pixel_fade_low_total,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=1000 -DHOST_TOTAL_LIMIT=1000))
$(eval $(call fixture,pixel_fade_zero_cap,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=1))
$(eval $(call fixture,pixel_fade_dim_cap,-DHOST_KIND=7 -DHOST_KIND_TOKEN=rgb_pixel -DCONFIG_LED_GLOBAL_BRIGHTNESS_PPTT=2000 -DHOST_TOTAL_LIMIT=7777))
clean:
	$(RM) $(BINS) $(FADE_BINS)
