def record_movement():
    Motor_Position_Data = {}
    Motor_Position_Data_List.clear()
    
    # Change to Position control mode
    for i in DXL_ID:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
    print("ALL MOTOR SET TO VELOCITY MODE! ")
        
    
    while recording == True:
        print("Start recording movement.....",end='\r')
        ## Read present position
        Motor_Position_Data  = {}
        
        for i in DXL_ID:
            Motor_Position_Data[f'M{i}'], dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION)
            
            if (Motor_Position_Data[f'M{i}'] > 4095) and (Motor_Position_Data[f'M{i}'] < 4200000000):
               Motor_Position_Data[f'M{i}'] = Motor_Position_Data[f'M{i}'] - 4095
            if Motor_Position_Data[f'M{i}'] > 4200000000:
                Motor_Position_Data[f'M{i}'] = Motor_Position_Data[f'M{i}'] - 4294967295 + 4095
        time.sleep(0.01)
        
        Motor_Position_Data_List.append(Motor_Position_Data)
        json_position_data = json.dumps(Motor_Position_Data_List, indent=4)
        
    with open("Motor_Position.json", "w") as outfile:
        outfile.write(json_position_data)
        
    print("Interrupt recording movement")
    print("Movement stored")
    sys.exit()

def replay_movement():
    with open("Motor_Position.json") as openfile:
        Motor_read_list = json.load(openfile)
        
    print("Start replaying movement...")

    for item in Motor_read_list:
        for i in DXL_ID:
            packetHandler.write4ByteTxRx(portHandler, i, ADDR_GOAL_POSITION, item[f'M{i}'])
            Motor_Pos_in_Replaying, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, i, ADDR_PRESENT_POSITION)
            print(f"M{i} Replay  Pos: {item[f'M{i}']}")
            print(f"M{i} Present Pos: {Motor_Pos_in_Replaying}")
        time.sleep(0.01)

    for i in DXL_ID:
        packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, DISABLE)
        if replaying == False:
            print("Interrupt replaying movement")
            sys.exit()
    print("Replay successfull!")
