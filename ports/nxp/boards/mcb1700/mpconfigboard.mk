USB_VID 						= 0xC251
USB_PID 						= 0x2201
USB_PRODUCT 					= "Keil MCB1700"
USB_MANUFACTURER 				= "Arm Keil"

CFLAGS += -DBOARD_MCB1700=1

CHIP_VARIANT                  	= LPC1768
CHIP_FAMILY                     = LPC1700
BOARD                           = mcb1700

BOARD_I2C_CONFIG                = 1
LONGINT_IMPL                    = NONE

CIRCUITPY_STDIO                 = 1
CIRCUITPY_CANIO                 = 1

CIRCUITPY_FULL_BUILD            = 0
CIRCUITPY_AUDIOBUSIO            = 0
CIRCUITPY_AUDIOIO               = 0
CIRCUITPY_AUDIOPWM              = 0
CIRCUITPY_DISPLAYIO             = 0
CIRCUITPY_I2CTARGET             = 0
CIRCUITPY_COUNTIO               = 0
CIRCUITPY_FREQUENCYIO           = 0
CIRCUITPY_NEOPIXEL_WRITE        = 0
CIRCUITPY_PULSEIO               = 0
CIRCUITPY_PWMIO                 = 0
CIRCUITPY_ROTARYIO              = 0
CIRCUITPY_RTC                   = 0
CIRCUITPY_USB_CDC               = 1
CIRCUITPY_TOUCHIO               = 0
CIRCUITPY_USB_VENDOR            = 0
CIRCUITPY_USB                   = 1
CIRCUITPY_STORAGE               = 1

# SPI_FLASH_FILESYSTEM = 1
# QSPI_FLASH_FILESYSTEM = 1
INTERNAL_FLASH_FILESYSTEM       = 0
DISABLE_FILESYSTEM              = 1

# Typically the first module to create
CIRCUITPY_MICROCONTROLLER		= 1
# Typically the second module to create
# CIRCUITPY_DIGITALIO = 0
# Other modules:
CIRCUITPY_ANALOGIO              = 1
CIRCUITPY_BUSIO                 = 1
CIRCUITPY_COUNTIO               = 0
CIRCUITPY_OS                    = 0
CIRCUITPY_NVM                   = 0
CIRCUITPY_SDCARDIO              = 0
CIRCUITPY_FRAMEBUFFERIO         = 0
# CIRCUITPY_I2CPERIPHERAL = 0
# Requires SPI, PulseIO (stub ok):
# CIRCUITPY_DISPLAYIO = 0

# These modules are implemented in shared-module/ - they can be included in
# any port once their prerequisites in common-hal are complete.
# Requires DigitalIO:
CIRCUITPY_BITBANGIO             = 0
# Requires DigitalIO
CIRCUITPY_GAMEPAD               = 0
# Requires neopixel_write or SPI (dotstar)
CIRCUITPY_PIXELBUF              = 0
# Requires OS
CIRCUITPY_RANDOM                = 0
# Requires OS, filesystem
# CIRCUITPY_STORAGE = 0
# Requires Microcontroller
CIRCUITPY_TOUCHIO               = 0
# Requires USB
CIRCUITPY_USB_HID               = 1
CIRCUITPY_USB_MIDI              = 0
CIRCUITPY_USB_MSC               = 1
# Does nothing without I2C
CIRCUITPY_REQUIRE_I2C_PULLUPS	= 0
# No requirements, but takes extra flash
CIRCUITPY_ULAB                	= 0

CIRCUITPY_BLEIO_HCI           	= 0

ifeq ($(CIRCUITPY_USB_MSC), 1)
CIRCUITPY_STORAGE             	= 1
endif

ifeq ($(CIRCUITPY_STORAGE), 1)
CIRCUITPY_OS                  	= 1
endif

# FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_OneWire
# FROZEN_MPY_DIRS += $(TOP)/frozen/Adafruit_CircuitPython_NeoPixel
