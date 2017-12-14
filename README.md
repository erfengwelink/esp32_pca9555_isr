
# esp32_pca9555_isr

+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
this project just simply shows how to drive I2C interrupt function(based on esp32 platform with IC pca9555).|
+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

1>hardware:

	   ESP32		 PCA9555
	+-------+		+-------+
	|	|		|	|
	| GPIO13|---------------|scl	|
	| GPIO14|---------------|sda	|
	| 	|		|	|
	| GPIO15|---------------|~INT	|
	|	|		|	|
	+-------+		+-------+




2>software:
	see the code.







note:

	1. only when pca9555 ios configured input mode the interrupt trigger is valid.
