import time
import rclpy
import DR_init
import socket

def get_host_ip():
    try:
        # Get the hostname of the current machine.
        hostname = socket.gethostname()
        
        # Get the IP address associated with the hostname.
        ip_address = socket.gethostbyname(hostname)
        return ip_address
    except socket.error:
        # Handle cases where the hostname cannot be resolved or other socket errors.
        return None

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    print("test11")
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            DR_MV_ORI_RADIAL,
            DR_TOOL,
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            movejx,
            movec,
            movesx,
            movesj,
            amovej,
            amovesj,
            move_periodic,
            amove_periodic,
            get_current_posx,
            get_current_posj,
            task_compliance_ctrl,
            release_compliance_ctrl,
            set_desired_force,
            release_force
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    def get_pos():
        posxTemp, _ = get_current_posx()
        posjTemp = get_current_posj()
        print(f"posj : {posjTemp}")
        print(f"posx : {posxTemp}")
        return 
    
    def wait_digital_input(sig_num):
        while not get_digital_input(sig_num):
        # while get_digital_input(sig_num):
            time.sleep(0.5)
            print(f"Wait for digital input: {sig_num}")
            pass

    def sdo(num = 1):
        print(f"set for digital output {num} for grip")
        output = [1,2,3,4]
        output.pop(num-1)
        for i in output:
            set_digital_output(i, OFF)
            print(f"set for digital output {i} OFF")
            time.sleep(0.2)
        set_digital_output(num, ON)
        print(f"set for digital output {num} ON")
        time.sleep(0.5)
        # wait_digital_input(sig_num=num)
        print(f"set digital {num} Done")
        time.sleep(3)

    def soft_grip():
        sdo(2)

    def grip():
        sdo(1)

    def full_release():
        sdo(3)

    def release():
        sdo(4)

    def pickupUsedCup():
        posjClientUp = posj([17.86, 50.70, 78.12, 76.39, -73.21, 40.24])
        posxClientUp = posx([793.22, -74.71, 160.25, 129.52, -90.63, -90.41])
        posjClientDown = posj([17.86, 60.50, 77.33, 73.95, -75.52, 49.29])
        posxClientDown = posx([793.22, -74.71, 60.25, 129.52, -90.63, -90.41])

        posjJunkUp = posj([52.32059097,  53.40830231,  85.36701202,  61.57935333, 112.33535767, -54.39147949])
        posxJunkUp = posx([-7.75, 469.36, 102.44, 178.11, 90.0, 90.01])

        posxRubbishDown = [92.25, 469.36, -22.18, 178.11, 90.0, 90.01]
    
        posjRubbishUp = [45.19, 60.01, 70.12, 59.05, 121.07, -48.79]
        posxRubbishUp = [92.25, 469.36, 102.44, 178.11, 90.0, 90.01]

        movej(JReady, vel=VELOCITY, acc=ACC)
        print("Move to Returned Cup")
        movej(posjClientUp, vel=VELOCITY, acc=ACC)
        full_release()

        print("Down to Returned Cup")
        movel(posxClientDown, vel=VELOCITY, acc=ACC)
        grip()
        movel(posxClientUp, vel=VELOCITY, acc=ACC)
        
        print("Move to Junk Zone")
        # movej([18.26, 49.10, 41.70, 89.42, -71.21, 0.85], vel=VELOCITY, acc=ACC)
        # movej([18.26, 49.11, 41.69, 89.42, 108.79, 0.84], vel=VELOCITY, acc=ACC)
        # movej(posjJunkUp, vel=VELOCITY, acc=ACC)
        posesjTemp = [
            posj([18.26, 49.10, 41.70, 89.42, -71.21, 0.85]),
            posj([18.26, 49.11, 41.69, 89.42, 108.79, 0.84]),
            posjJunkUp
        ]
        movesj(posesjTemp, vel=VELOCITY, acc=ACC)
        movej([0., 0., 0., 0., 0., 180.], vel=VELOCITY, acc=ACC, mod=1)     # Flip Cup

        print("Move to Rubbish")
        # movej(posjRubbishUp, vel=VELOCITY, acc=ACC)
        movel([100., 0., 0., 0., 0., 0.], vel=VELOCITY, acc=ACC, mod=1)

        print("Down to Rubbish")
        # movel(posxRubbishDown, vel=VELOCITY, acc=ACC)
        movel([0., 0., -124.62, 0., 0., 0.], vel=VELOCITY, acc=ACC, mod=1)
        release()
        # movel(posxRubbishUp, vel=VELOCITY, acc=ACC)
        movel([0., 0., 124.62, 0., 0., 0.], vel=VELOCITY, acc=ACC, mod=1)

        movej(JReady, vel=VELOCITY, acc=ACC)

    def washUsedCup():
        posxRubbishDown = [92.25, 469.36, -22.18, 178.11, 90.0, 90.01]
    
        posjRubbishUp = [45.19, 60.01, 70.12, 59.05, 121.07, -48.79]
        posxRubbishUp = [92.25, 469.36, 102.44, 178.11, 90.0, 90.01]

        posjSinkUp = posj([196.07, 40.84, 111.50, 124.44, 73.30, 112.53])
        posxSinkUp = posx([-547.03, -455.58, 120.00, 68.55, -90., 90.])

        # posjSinkDown = posj([148.31, 57.92, 81.98, 69.66,112.21, -54.76])
        posxSinkDown = posx([-547.03, -455.58, -3.0, 68.55, -90., 90.])

        posjSinkVerUp = posj([218.34, 56.23, 16.91, -0.49, 106.77, 126.31])
        posxSinkVerUp = posx([-540.24, -434.96, 119.39, 156.85, -180., 64.77])
        
        posjUsableUpBefore = posj([269.793,  21.306, 131.9542, 179.769, 63.261, -89.896])
        posxUsableUpBefore = posx([5.114, -435.935, 189.261, 90., -90., -90.])

        posjUsableUp = posj([226.39, 45.40, 100.02, 72.87, 101.19, 112.95])
        posxUsableUp = posx([-16.36, -505.15, 120., 157.03, -90., 90.])

        posjUsableDown = posj([226.39, 59.89, 96.72, 71.00, 97.69, 112.17])
        posxUsableDown = posx([-16.36-10, -505.15, 3.03, 157.03, -90., 90.])

        # CupInfo
        cupRadius = 30.

        # CupsStackInfo
        numRubbishedCup = 1
        numUsableCup = 10
        rubbishedCupsBaseHeight = -94.6
        usableCupsBaseHeight = 100.0
        cupHeight = 92.
        cupStep = 11.

        posxCircleStart = posx([-540.24, -434.96+cupRadius, 119.39, 156.85, -180., 64.77])
        posesCircleDown = [
            posx([-540.24, -434.96+cupRadius, 30., 156.85, -180., 64.77]),
            posx([-579.110, -439.320, 30., 37.79, -180., -141.6]),
            posx([-546.62, -475.330, 30., 41.05, 180., 131.22]),
            posx([-509.540, -442.430, 30., 40.47, -180., 40.32]),
            posx([-553.730, -401.470, 30., 35.48, 180., -55.23])
        ]
        movej(JReady, vel=VELOCITY, acc=ACC)
        
        print("Move to Rubbish Up")
        movej([45., 0., 0., 0., 0., 0.], vel=VELOCITY, acc=ACC, mod=1)
        movej(posjRubbishUp, vel=VELOCITY, acc=ACC)
        full_release()

        print("Move to Rubbish Down")
        movel(posxRubbishDown, vel=VELOCITY, acc=ACC)
        grip()

        print("Move to Rubbish Up")
        movel(posxRubbishUp, vel=VELOCITY, acc=ACC)

        print("Move to Sink Up")
        movej(posjSinkUp, vel=VELOCITY, acc=ACC)
        movel(posxSinkUp, vel=VELOCITY, acc=ACC)

        print("Move to Sink Down")
        movel(posxSinkDown, vel=VELOCITY, acc=ACC)
        full_release()

        print("Move to Sink Up")
        movel(posxSinkUp, vel=VELOCITY, acc=ACC)


        print("Move to Sink Vertical Up")
        movej(posjSinkVerUp, vel=VELOCITY, acc=ACC)
        movel(posxSinkVerUp, vel=VELOCITY, acc=ACC)
        soft_grip()

        print("Brush Cup Start")
        movel(posxCircleStart, vel=VELOCITY, acc=ACC)
        movel(posesCircleDown[0], vel=VELOCITY, acc=ACC)
        movesx(posesCircleDown[1:5], vel=VELOCITY, acc=ACC)
        
        for i in range(len(posesCircleDown)):
            posesCircleDown[i][2] = 10.
        movel(posesCircleDown[4], vel=VELOCITY, acc=ACC)
        movesx(posesCircleDown[3::-1], vel=VELOCITY, acc=ACC)
        
        print("Brush Cup End")
        movel(posxCircleStart, vel=VELOCITY, acc=ACC)
        full_release()
        movej(posjSinkUp, vel=VELOCITY, acc=ACC)
        movel(posxSinkDown, vel=VELOCITY, acc=ACC)
        grip()
        movel(posxSinkUp, vel=VELOCITY, acc=ACC)

        print("Move to UsableZone")
        movej(posjUsableUp, vel=VELOCITY, acc=ACC)
        movel(posxUsableDown, vel=VELOCITY, acc=ACC)
        full_release()
        movel(posxUsableUp, vel=VELOCITY, acc=ACC)

        print("Return Home")
        movej(JReady, vel=VELOCITY, acc=ACC)

    def shaking():
        ########### Shaker Ready ###########
        posjShakerStandbyUp = posj([14.91, 34.94, 88.95, 100.21, -103.02, 36.98])
        posxShakerStandbyUp = posx([511.62, -200.24, 258.94, 88.75, -91.23, -88.29])

        posjShakerStandbyDown = posj([14.91, 54.77, 91.35, 104.21, -98.26, 59.08])
        posxShakerStandbyDown = posx([511.62, -200.24, 58.94, 88.75, -91.23, -88.29])

        posxShakerReadyUp = posx([368.55, 100.36, 288.32, 90., -90., -90.])

        posjShakerReadyDown = posj([49.73, 56.10, 83.16, 131.58, -120.49, 60.86])
        posxShakerReadyDown = posx([368.55, 100.36, 88.32, 90., -90., -90.])

        posjShakerReadyVerUp = posj([17.84, 3.17, 80.75, -0.35, 95.31, 108.28])
        posxShakerReadyVerUp = posx([371.87, 126.70, 233.23, 175.07, -179.16, -94.82])

        posjShakerReadyVerDown = posj([17.84, 3.65, 95.87, -0.35, 79.71, 108.37])
        posxShakerReadyVerDown = posx([371.88, 126.74, 133.26, 175.19, -179.16, -94.71])

        posjHatUp = posj([-5.47, -3.38, 87.87, -0.02, 94.53, 84.75])
        posxHatUp = posx([346.78, -24.64, 229.54, 172.66, -179.02, -97.49])

        posjHatDown = posj([-5.44, 1.98, 114.71, -0.05, 62.33, 84.81])
        posxHatDown = posx([346.78, -24.64, 29.54, 172.66, -179.02, -97.49])

        posjShakerGripBeforeUp = posj([51.92, 52.45, 70.76, 128.21, -133.09, 139.69])
        posxShakerGripBeforeUp = posx([365.46, 155.60, 181.84, 87.10, -90.41, 0.51])

        posjShakerGripBeforeDown = posj([51.92, 61.49, 70.72, 133.94, -127.16, 148.56])
        posxShakerGripBeforeDown = posx([365.46, 155.60, 81.84, 87.10, -90.41, 0.51])

        posjShakerGrip = posj([49.50, 58.01, 78.96, 133.78, -122.63, 151.19])
        posxShakerGrip = posx([364.45, 113.22, 81.68, 87.13, -90.40, 0.45])

        posjShakerGripUp = posj([49.42, 48.53, 78.84, 128.54, -128.93, 142.25])
        posxShakerGripUp = posx([365.44, 113.21, 181.68, 87.13, -90.40, 0.45])
        
        print("Move to Shaker Standby Up")
        movej(posjShakerStandbyUp, vel=VELOCITY, acc=ACC)
        full_release()

        print("Move to Shaker Standby Down")
        movel(posxShakerStandbyDown, vel=VELOCITY, acc=ACC)
        grip()

        print("Move to Shaker Standby Up")
        movel(posxShakerStandbyUp, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Ready Up")
        movel(posxShakerReadyUp, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Ready Down")
        movel(posxShakerReadyDown, vel=VELOCITY, acc=ACC)
        full_release()

        print("Move to Shaker Ready Up")
        movel(posxShakerReadyUp, vel=VELOCITY, acc=ACC)

        print("Move to Hat Up")
        movej(posjHatUp, vel=VELOCITY, acc=ACC)

        print("Move to Hat Down")
        movel(posxHatDown, vel=VELOCITY, acc=ACC)
        release()
        grip()

        print("Move to Hat Up")
        movel(posxHatUp, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Ready Vertical Up")
        movel(posxShakerReadyVerUp, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Ready Vertical Down")
        movel(posxShakerReadyVerDown, vel=VELOCITY, acc=ACC)
        full_release()      # Release Hat

        print("Move to Shaker Ready Vertical Up")
        movel(posxShakerReadyVerUp, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Grip BeforeUp")
        movej(posjShakerGripBeforeUp, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Grip BeforeDown")
        movel(posxShakerGripBeforeDown, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Grip")
        movel(posxShakerGrip, vel=VELOCITY, acc=ACC)
        grip()          # Grip Shaker + Hat
        movel(posxShakerGripUp, vel=VELOCITY, acc=ACC)

        ########### Shaking Start ###########
        movej(posjShakerGripUp, vel=VELOCITY, acc=ACC)
        posjShakingIdle = posj([0., 0., 90., 90., 0., 90.])
        movej(posjShakingIdle, vel=VELOCITY, acc=ACC)
        print("Shaking Start!")
        move_periodic([0., 0., 0., 15., 20., 0.], [0., 0., 0., 0., 1, 0.], 0.5, 5, ref=DR_TOOL)
        movej([0., 0., 0., 90., 0., 0.], vel=VELOCITY, acc=ACC, mod=1)
        move_periodic([0., 0., 0., 15., 20., 0.], [0., 0., 0., 1., 0, 0.], 1., 5, ref=DR_TOOL)
        
        print("Shaking End")
        movej(posjShakerGripUp, vel=VELOCITY, acc=ACC)

        ###########  Shaking End  ###########
        movel(posxShakerGrip, vel=VELOCITY, acc=ACC)
        full_release()    # Release Shaker + Hat

        print("Move to Shaker Grip BeforeDown")
        movel(posxShakerGripBeforeDown, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Grip BeforeUp")
        movel(posxShakerGripBeforeUp, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Ready Vertical Up")
        movej(posjShakerReadyVerUp, vel=VELOCITY, acc=ACC)

        print("Move to Shaker Ready Vertical Down")
        movel(posxShakerReadyVerDown, vel=VELOCITY, acc=ACC)
        grip()          # Grip Hat

        print("Move to Shaker Ready Vertical Up")
        movel(posxShakerReadyVerUp, vel=VELOCITY, acc=ACC)

        print("Move to Hat Up")
        movel(posxHatUp, vel=VELOCITY, acc=ACC)

        print("Move to Hat Down")
        movel(posxHatDown, vel=VELOCITY, acc=ACC)
        release()       # Release Hat

        print("Move to Hat Up")
        movel(posxHatUp, vel=VELOCITY, acc=ACC)

        movej(JReady, vel=VELOCITY, acc=ACC)

    def provideMenu():
        posjShakerReadyUp = posj([49.73, 38.46, 79.29, 118.24, -132.98, 38.85])
        posxShakerReadyUp = posx([368.55, 100.36, 288.32, 90., -90., -90.])

        posjShakerReadyDown = posj([49.73, 56.10, 83.16, 131.58, -120.49, 60.86])
        posxShakerReadyDown = posx([368.55, 100.36, 88.32, 90., -90., -90.])

        posjClientUp = posj([17.86, 50.70, 78.12, 76.39, -73.21, 40.24])
        posxClientUp = posx([793.22, -74.71, 160.25, 129.52, -90.63, -90.41])
        posjClientDown = posj([17.86, 60.50, 77.33, 73.95, -75.52, 49.29])
        posxClientDown = posx([793.22, -74.71, 60.25, 129.52, -90.63, -90.41])
        
        print("Grip Menu")
        movej(posjShakerReadyUp, vel=VELOCITY, acc=ACC)
        full_release()
        movel(posxShakerReadyDown, vel=VELOCITY, acc=ACC)
        grip()
        movel(posxShakerReadyUp, vel=VELOCITY, acc=ACC)

        print("Move to Client")
        movel(posxClientUp, vel=VELOCITY, acc=ACC)
        movel(posxClientDown, vel=VELOCITY, acc=ACC)
        full_release()
        movel(posxClientUp, vel=VELOCITY, acc=ACC)

        movej(JReady, vel=VELOCITY, acc=ACC)

    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    JReady = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])

    if not rclpy.ok():
        rclpy.shutdown()
        return
    print(get_host_ip())
    release()
    movej(JReady, vel=VELOCITY, acc=ACC)

    # pickupUsedCup()
    washUsedCup()
    # shaking()
    # provideMenu()
    rclpy.shutdown()

if __name__ == "__main__":
    main()