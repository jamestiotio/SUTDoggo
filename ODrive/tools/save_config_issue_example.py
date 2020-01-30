import odrive
from fibre import Logger, Event
from odrive.utils import OperationAbortedException
from fibre.protocol import ChannelBrokenException

def get_odrive(shutdown_token):
    print('Looking for ODrive...',end='')
    odrv = odrive.find_any(search_cancellation_token=app_shutdown_token, channel_termination_token=app_shutdown_token)
    print('Found.')
    return odrv

def reboot_odrive(odrv):
    """
    Reboot odrive
    """
    try:
        odrv.reboot()
    except ChannelBrokenException:
        print('Lost connection because of reboot...')

app_shutdown_token = Event()
try:
    odrv0 = get_odrive(app_shutdown_token)
    print('Erasing configuration...',end='')
    odrv0.erase_configuration()
    print('Done.')
    reboot_odrive(odrv0)
    odrv0 = get_odrive(app_shutdown_token)

    print('*** Calling save_configuration() 3 times...',end='')
    odrv0.config.brake_resistance = 0.0
    odrv0.save_configuration()
    odrv0.config.brake_resistance = 1.0
    odrv0.save_configuration()
    odrv0.config.brake_resistance = 2.0
    odrv0.save_configuration()
    odrv0.config.brake_resistance = 3.0
    odrv0.save_configuration()
    print('Done.')

    reboot_odrive(odrv0)

    odrv0 = get_odrive(app_shutdown_token)
    print('Brake resistance (should be 2.0): ',end='')
    print(odrv0.config.brake_resistance)

    # init_odrive(odrv0)
except OperationAbortedException:
    logger.info("Operation aborted.")
finally:
    app_shutdown_token.set()