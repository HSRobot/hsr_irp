# 由 trajectory_execution.launch.xml 控制ros control的启动和停止
先有move_group.launch 启动
加载trajectory_execution.launch.xml
后再在各个控制器里面进行加载yaml 和controller plugin
