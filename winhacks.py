#from pyvesc.VESCMotor.messages import GetValues, SetRPM, SetCurrent, SetRotorPositionMode, GetRotorPosition
import pyvesc
import vlc
import serial
import time
import struct
from enum import IntEnum


class VedderCmd(IntEnum):
    COMM_FW_VERSION = 0
    COMM_JUMP_TO_BOOTLOADER = 1
    COMM_ERASE_NEW_APP = 2
    COMM_WRITE_NEW_APP_DATA = 3
    COMM_GET_VALUES = 4
    COMM_SET_DUTY = 5
    COMM_SET_CURRENT = 6
    COMM_SET_CURRENT_BRAKE = 7
    COMM_SET_RPM = 8
    COMM_SET_POS = 9
    COMM_SET_HANDBRAKE = 10
    COMM_SET_DETECT = 11
    COMM_SET_SERVO_POS = 12
    COMM_SET_MCCONF = 13
    COMM_GET_MCCONF = 14
    COMM_GET_MCCONF_DEFAULT = 15
    COMM_SET_APPCONF = 16
    COMM_GET_APPCONF = 17
    COMM_GET_APPCONF_DEFAULT = 18
    COMM_SAMPLE_PRINT = 19
    COMM_TERMINAL_CMD = 20
    COMM_PRINT = 21
    COMM_ROTOR_POSITION = 22
    COMM_EXPERIMENT_SAMPLE = 23
    COMM_DETECT_MOTOR_PARAM = 24
    COMM_DETECT_MOTOR_R_L = 25
    COMM_DETECT_MOTOR_FLUX_LINKAGE = 26
    COMM_DETECT_ENCODER = 27
    COMM_DETECT_HALL_FOC = 28
    COMM_REBOOT = 29
    COMM_ALIVE = 30
    COMM_GET_DECODED_PPM = 31
    COMM_GET_DECODED_ADC = 32
    COMM_GET_DECODED_CHUK = 33
    COMM_FORWARD_CAN = 34
    COMM_SET_CHUCK_DATA = 35
    COMM_CUSTOM_APP_DATA = 36
    COMM_NRF_START_PAIRING = 37
    COMM_GPD_SET_FSW = 38
    COMM_GPD_BUFFER_NOTIFY = 39
    COMM_GPD_BUFFER_SIZE_LEFT = 40
    COMM_GPD_FILL_BUFFER = 41
    COMM_GPD_OUTPUT_SAMPLE = 42
    COMM_GPD_SET_MODE = 43
    COMM_GPD_FILL_BUFFER_INT8 = 44
    COMM_GPD_FILL_BUFFER_INT16 = 45
    COMM_GPD_SET_BUFFER_INT_SCALE = 46
    COMM_GET_VALUES_SETUP = 47
    COMM_SET_MCCONF_TEMP = 48
    COMM_SET_MCCONF_TEMP_SETUP = 49
    COMM_GET_VALUES_SELECTIVE = 50
    COMM_GET_VALUES_SETUP_SELECTIVE = 51
    COMM_EXT_NRF_PRESENT = 52
    COMM_EXT_NRF_ESB_SET_CH_ADDR = 53
    COMM_EXT_NRF_ESB_SEND_DATA = 54
    COMM_EXT_NRF_ESB_RX_DATA = 55
    COMM_EXT_NRF_SET_ENABLED = 56
    COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP = 57
    COMM_DETECT_APPLY_ALL_FOC = 58
    COMM_JUMP_TO_BOOTLOADER_ALL_CAN = 59
    COMM_ERASE_NEW_APP_ALL_CAN = 60
    COMM_WRITE_NEW_APP_DATA_ALL_CAN = 61
    COMM_PING_CAN = 62
    COMM_APP_DISABLE_OUTPUT = 63
    COMM_TERMINAL_CMD_SYNC = 64
    COMM_GET_IMU_DATA = 65
    COMM_BM_CONNECT = 66
    COMM_BM_ERASE_FLASH_ALL = 67
    COMM_BM_WRITE_FLASH = 68
    COMM_BM_REBOOT = 69
    COMM_BM_DISCONNECT = 70
    COMM_BM_MAP_PINS_DEFAULT = 71
    COMM_BM_MAP_PINS_NRF5X = 72
    COMM_ERASE_BOOTLOADER = 73
    COMM_ERASE_BOOTLOADER_ALL_CAN = 74
    COMM_PLOT_INIT = 75
    COMM_PLOT_DATA = 76
    COMM_PLOT_ADD_GRAPH = 77
    COMM_PLOT_SET_GRAPH = 78
    COMM_GET_DECODED_BALANCE = 79
    COMM_BM_MEM_READ = 80
    COMM_WRITE_NEW_APP_DATA_LZO = 81
    COMM_WRITE_NEW_APP_DATA_ALL_CAN_LZO = 82
    COMM_BM_WRITE_FLASH_LZO = 83
    COMM_SET_CURRENT_REL = 84
    COMM_CAN_FWD_FRAME = 85
    COMM_SET_BATTERY_CUT = 86
    COMM_SET_BLE_NAME = 87
    COMM_SET_BLE_PIN = 88
    COMM_SET_CAN_MODE = 89

class VESCMessage(type):
    """ Metaclass for VESC messages.
    This is the metaclass for any VESC message classes. A VESC message class must then declare 2 static attributes:
    id: unsigned integer which is the identification number for messages of this class
    fields: list of tuples. tuples are of size 2, first element is the field name, second element is the fields type
            the third optional element is a scalar that will be applied to the data upon unpack
    format character. For more info on struct format characters see: https://docs.python.org/2/library/struct.html
    """
    _msg_registry = {}
    _endian_fmt = '!'
    _id_fmt = 'B'
    _can_id_fmt = 'BB'
    _comm_forward_can = 33
    _entry_msg_registry = None

    def __init__(cls, name, bases, clsdict):
        cls.can_id = None
        msg_id = clsdict['id']
        # make sure that message classes are final
        for klass in bases:
            if isinstance(klass, VESCMessage):
                raise TypeError("VESC messages cannot be inherited.")
        # check for duplicate id
        if msg_id in VESCMessage._msg_registry:
            raise TypeError("ID conflict with %s" % str(VESCMessage._msg_registry[msg_id]))
        else:
            VESCMessage._msg_registry[msg_id] = cls
        # initialize cls static variables
        cls._string_field = None
        cls._fmt_fields = ''
        cls._field_names = []
        cls._field_scalars = []
        for field, idx in zip(cls.fields, range(0, len(cls.fields))):
            cls._field_names.append(field[0])
            if len(field) >= 3:
                cls._field_scalars.append(field[2])
            if field[1] is 's':
                # string field, add % so we can vary the length
                cls._fmt_fields += '%u'
                cls._string_field = idx
            cls._fmt_fields += field[1]
        cls._full_msg_size = struct.calcsize(cls._fmt_fields)
        # check that at most 1 field is a string
        string_field_count = cls._fmt_fields.count('s')
        if string_field_count > 1:
            raise TypeError("Max number of string fields is 1.")
        if 'p' in cls._fmt_fields:
            raise TypeError("Field with format character 'p' detected. For string field use 's'.")
        super(VESCMessage, cls).__init__(name, bases, clsdict)

    def __call__(cls, *args, **kwargs):
        instance = super(VESCMessage, cls).__call__()
        if 'can_id' in kwargs:
            instance.can_id = kwargs['can_id']
        else:
            instance.can_id = None
        if args:
            if len(args) != len(cls.fields):
                raise AttributeError("Expected %u arguments, received %u" % (len(cls.fields), len(args)))
            for name, value in zip(cls._field_names, args):
                setattr(instance, name, value)
        return instance

    @staticmethod
    def msg_type(id):
        return VESCMessage._msg_registry[id]

    @staticmethod
    def unpack(msg_bytes):
        msg_id = struct.unpack_from(VESCMessage._endian_fmt + VESCMessage._id_fmt, msg_bytes, 0)
        msg_type = VESCMessage.msg_type(*msg_id)
        data = None
        if not (msg_type._string_field is None):
            # string field
            fmt_wo_string = msg_type._fmt_fields.replace('%u', '')
            fmt_wo_string = fmt_wo_string.replace('s', '')
            len_string = len(msg_bytes) - struct.calcsize(VESCMessage._endian_fmt + fmt_wo_string) - 1
            fmt_w_string = msg_type._fmt_fields % (len_string)
            data = struct.unpack_from(VESCMessage._endian_fmt + fmt_w_string, msg_bytes, 1)
        else:
            data = list(struct.unpack_from(VESCMessage._endian_fmt + msg_type._fmt_fields, msg_bytes, 1))
            for k, field in enumerate(data):
                try:
                    if msg_type._field_scalars[k] != 0:
                        data[k] = data[k]/msg_type._field_scalars[k]
                except (TypeError, IndexError) as e:
                    print("Error ecountered on field " + msg_type.fields[k][0])
                    print(e)
        msg = msg_type(*data)
        if not (msg_type._string_field is None):
            string_field_name = msg_type._field_names[msg_type._string_field]
            setattr(msg,
                    string_field_name,
                    getattr(msg, string_field_name).decode('ascii'))
        return msg

    @staticmethod
    def pack(instance, header_only=None):
        if header_only:
            if instance.can_id is not None:
                fmt = VESCMessage._endian_fmt + VESCMessage._can_id_fmt + VESCMessage._id_fmt
                values = (VESCMessage._comm_forward_can, instance.can_id, instance.id)
            else:
                fmt = VESCMessage._endian_fmt + VESCMessage._id_fmt
                values = (instance.id,)
            return struct.pack(fmt, *values)

        field_values = []
        if not instance._field_scalars:
            for field_name in instance._field_names:
                field_values.append(getattr(instance, field_name))
        else:
            for field_name, field_scalar in zip(instance._field_names, instance._field_scalars):
                field_values.append(int(getattr(instance, field_name) * field_scalar))
        if not (instance._string_field is None):
            # string field
            string_field_name = instance._field_names[instance._string_field]
            string_length = len(getattr(instance, string_field_name))
            field_values[instance._string_field] = field_values[instance._string_field].encode('ascii')
            values = ((instance.id,) + tuple(field_values))
            if instance.can_id is not None:
                fmt = VESCMessage._endian_fmt + VESCMessage._can_id_fmt + VESCMessage._id_fmt\
                      + (instance._fmt_fields % (string_length))
                values = (VESCMessage._comm_forward_can, instance.can_id) + values
            else:
                fmt = VESCMessage._endian_fmt + VESCMessage._id_fmt + (instance._fmt_fields % (string_length))
            return struct.pack(fmt, *values)
        else:
            values = ((instance.id,) + tuple(field_values))
            if instance.can_id is not None:
                fmt = VESCMessage._endian_fmt + VESCMessage._can_id_fmt + VESCMessage._id_fmt + instance._fmt_fields
                values = (VESCMessage._comm_forward_can, instance.can_id) + values
            else:
                fmt = VESCMessage._endian_fmt + VESCMessage._id_fmt + instance._fmt_fields
            return struct.pack(fmt, *values)
        
class GetValues(metaclass=VESCMessage):
    """ Gets internal sensor data
    """
    id = VedderCmd.COMM_GET_VALUES

    fields = [
        ('temp_fet', 'h', 10),
        ('temp_motor', 'h', 10),
        ('avg_motor_current', 'i', 100),
        ('avg_input_current', 'i', 100),
        ('avg_id', 'i', 100),
        ('avg_iq', 'i', 100),
        ('duty_cycle_now', 'h', 1000),
        ('rpm', 'i', 1),
        ('v_in', 'h', 10),
        ('amp_hours', 'i', 10000),
        ('amp_hours_charged', 'i', 10000),
        ('watt_hours', 'i', 10000),
        ('watt_hours_charged', 'i', 10000),
        ('tachometer', 'i', 1),
        ('tachometer_abs', 'i', 1),
        ('mc_fault_code', 'c', 0),
        ('pid_pos_now', 'i', 1000000),
        ('app_controller_id', 'c', 0),
        ('time_ms', 'i', 1),
    ]

# Set your serial port here (either /dev/ttyX or COMX)
serialport = '/dev/ttyACM0'

def get_values_example():
    
    music = vlc.MediaPlayer("soundv1balls.wav")
    
    with serial.Serial(serialport, baudrate=115200, timeout=0.05) as ser:
        try:
            while True:
                
                ser.write(pyvesc.encode_request(GetValues))
                
                if ser.in_waiting > 78:
                    response, consumed = pyvesc.decode(ser.read(78))
                    # Print out the values
                    cur = response.current_in
                    print(cur)
                    if cur != 655.35 and cur > 0.02 and music.get_state() != 3: # if music not playing and current in moving state
                        music.play()
                    elif not (cur != 655.35 and cur > 0.02) and music.get_state() != 4:
                        music.pause()
                    if music.get_state() == 6:
                        music.stop()
                        music.play()

                    

                time.sleep(0.1)

        except KeyboardInterrupt:
            print('penis')
            # Turn Off the VESC
            #ser.write(pyvesc.encode(SetCurrent(0)))


if __name__ == "__main__":
    get_values_example()