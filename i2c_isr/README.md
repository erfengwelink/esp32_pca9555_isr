# esp32_pca9555_isr

+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
this project just simply shows how to drive I2C interrupt function(based on esp32 platform with IC pca9555).|
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

1>hardware:

	   ESP32		         PCA9555
	+-------+		        +-------+
	|	    |		        |	    |
	| GPIO13|---------------|scl	|
	| GPIO14|---------------|sda	|
	| 	    |		        |	    |
	| GPIO15|---------------|~INT	|
	|	    |		        |	    |
	+-------+		        +-------+




2>software:
	see the code.

log:


I (0) cpu_start: App cpu up.
I (195) heap_init: Initializing. RAM available for dynamic allocation:
I (202) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM
I (208) heap_init: At 3FFB2990 len 0002D670 (181 KiB): DRAM
I (214) heap_init: At 3FFE0440 len 00003BC0 (14 KiB): D/IRAM
I (220) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM
I (227) heap_init: At 40089AA8 len 00016558 (89 KiB): IRAM
I (233) cpu_start: Pro cpu start user code
I (251) cpu_start: Starting scheduler on PRO CPU.
I (0) cpu_start: Starting scheduler on APP CPU.
I (252) PCA9555: pca9555_init ...
I (252) gpio: GPIO[15]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:0 
now_val:7f00 || last_val:ff00 || changed_bits: 8000
now_val:7f00 || last_val:ff00 || changed_bits: 8000
now_val:3f00 || last_val:ff00 || changed_bits: c000
now_val:3f00 || last_val:ff00 || changed_bits: c000
now_val:3f00 || last_val:ff00 || changed_bits: c000
now_val:3f00 || last_val:ff00 || changed_bits: c000
now_val:bf00 || last_val:ff00 || changed_bits: 4000
now_val:bf00 || last_val:ff00 || changed_bits: 4000
now_val:df00 || last_val:ff00 || changed_bits: 2000
now_val:df00 || last_val:ff00 || changed_bits: 2000
now_val:df00 || last_val:ff00 || changed_bits: 2000
now_val:df00 || last_val:ff00 || changed_bits: 2000
now_val:df00 || last_val:ff00 || changed_bits: 2000
now_val:df00 || last_val:ff00 || changed_bits: 2000
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:bf00 || last_val:ff00 || changed_bits: 4000
now_val:bf00 || last_val:ff00 || changed_bits: 4000
now_val:bf00 || last_val:ff00 || changed_bits: 4000
now_val:bf00 || last_val:ff00 || changed_bits: 4000
now_val:ff00 || last_val:ff00 || changed_bits: 00
now_val:bf00 || last_val:ff00 || changed_bits: 4000



*note*:

	1. only when pca9555 ios configured input mode the interrupt trigger is valid.
