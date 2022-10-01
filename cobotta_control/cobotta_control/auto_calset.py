# -*- coding:utf-8 -*-

# COBOTTA
# AutoCAL sequence

# b-cap Lib URL
# https://github.com/DENSORobot/orin_bcap


from pybcapclient import bcapclient


class AutoCalset(object):

    def __init__(self):
        # set IP Address , Port number and Timeout of connected RC8
        self.host = "192.168.0.2"
        self.port = 5007
        self.timeout = 2000

        # set Parameter
        self.Name = ""
        self.Provider="CaoProv.DENSO.VRC"
        self.Machine = ("localhost")
        self.Option = ("")

        self.comp = 1
        self.loopflg = True
        self.ESC = 0x1B  # [ESC] virtual key code

    def start_bcap(self):
        # Connection processing of tcp communication
        self.m_bcapclient = bcapclient.BCAPClient(
            self.host,self.port,self.timeout
        )

        # start b_cap Service
        self.m_bcapclient.service_start("")

    def connect_rc8(self):
        # Connect to RC8 (RC8(VRC)provider)
        hCtrl = self.m_bcapclient.controller_connect(
            self.Name,self.Provider,self.Machine,self.Option
        )

        return hCtrl

    def stop_rc8(self):
        self.m_bcapclient.service_stop()

    def auto_calset(self):
        self.start_bcap()

        try:
            # Connect to RC8 (RC8(VRC)provider) , Get Controller Handle
            hCtrl = self.connect_rc8()

            print("Connect RC8")
            # Get Robot Handle
            hRobot = self.m_bcapclient.controller_getrobot(hCtrl, "Arm", "")

            # VirtualTP > arm auxiliary function > terms of use > 
            # 252 CALSET at startup = 1
            self.m_bcapclient.robot_execute(hRobot, "AutoCal", "")

            state = self.m_bcapclient.robot_execute(hRobot, "GetMotionPreparationState", "")
            print("AutoCal Status:{}".format(state))

            # COBOTTA Version 2.8.0～
            # Comment out If version 2.7.X or lower
            # self.m_bcapclient.robot_execute(hRobot, "ManualResetPreparation", "")

            self.m_bcapclient.robot_execute(hRobot, "MotionPreparation", "")
            self.m_bcapclient.robot_execute(hRobot, "TakeArm")
            self.m_bcapclient.robot_execute(hRobot, "ExtSpeed", 100)
            self.m_bcapclient.robot_move(hRobot, 1, "@P J(0,0,90,0,90,0)")

        except Exception as e:
            print('=== ERROR Description ===')
            if str(type(e)) == "<class 'web_app.pybcapclient.orinexception.ORiNException'>":
                print(e)
                errorcode_int = int(str(e))
                if errorcode_int < 0:
                    errorcode_hex = format(errorcode_int & 0xffffffff, 'x')
                else:
                    errorcode_hex = hex(errorcode_int)
                print("Error Code : 0x" + str(errorcode_hex))
                error_description = self.m_bcapclient.controller_execute(
                    hCtrl, "GetErrorDescription", errorcode_int)
                print("Error Description : " + error_description)
            else:
                print(e)

        finally:
            # DisConnect
            if(hRobot != 0):
                self.m_bcapclient.robot_release(hRobot)
                print("Release Robot Handle")
            # End If
            if(hCtrl != 0):
                self.m_bcapclient.controller_disconnect(hCtrl)
                print("Release Controller")
            # End If
            self.stop_rc8()


if __name__ == '__main__':
        robot = AutoCalset()
        robot.auto_calset()