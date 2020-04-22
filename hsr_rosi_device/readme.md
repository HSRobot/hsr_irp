-- Date
記錄相關的出錯 出現
[ERROR] [1587556529.491409911]: Unable to identify any set of controllers that can actuate the specified joints: [ joint_1 joint_2 joint_3 joint_4 joint_5 joint_6 ]
[ERROR] [1587556529.491598441]: Known controllers and their joints:

這種錯誤是由於 joint_action加載的軸的名稱不一樣而導致的,將軸的名稱修改會回來即可 如   <group ns="UR52"/> 那麼 在control.yaml文件裏面需要需改一下 action_ns: UR52/joint_trajectory_action
