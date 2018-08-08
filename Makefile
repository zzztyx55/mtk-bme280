ccflags-y += -I$(srctree)/drivers/misc/mediatek/temperature/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/humidity/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/barometer/inc
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

obj-y := bme280.o

