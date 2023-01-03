### common file
obj-$(CONFIG_NUBIA_SENSORS)                             += sensor_common.o

#---------------------------PS and Light SENSOR----------------------------------#
obj-$(CONFIG_NUBIA_SENSORS_STK3337)                     += stk3337/
ifeq ($(TARGET_NUBIA_NX651J_V1C), true)
     obj-y                    += mlx90632/
endif
