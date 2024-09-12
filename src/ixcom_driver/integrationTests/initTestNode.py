import rclpy
from threading import Thread
from ..ixcom_driver.ixcom_driver_subscriber_integrationTests import DataSubscriberIntegrationTests
from ..integrationTests.config import settings

def NodeSpin():
    try:
        rclpy.spin(dSub)
    except KeyboardInterrupt:
        dSub.destroy_node()
        exit(0)

# def reloadSettings(path):
#     settings.reload(path)

rclpy.init()
dSub = DataSubscriberIntegrationTests(True)
dSub.get_config_file(0)
dSub.rx_data(settings.PathToPythonConfigFile)

NodeSpinThread = Thread(target=NodeSpin,daemon=True)
NodeSpinThread.start()
