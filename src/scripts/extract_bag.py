import os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import pandas as pd

def read_bag(bag_path, target_topic):
    # 1. 检查 bag 路径是否存在
    if not os.path.exists(bag_path):
        print(f"错误：找不到 bag 文件夹 -> {bag_path}")
        return []

    # 2. 初始化 reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader.open(storage_options, converter_options)

    # 3. 获取数据类型映射
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    if target_topic not in type_map:
        print(f"错误：话题 '{target_topic}' 不存在于包中！")
        print(f"当前包包含的可用话题有: {list(type_map.keys())}")
        return []

    msg_type_str = type_map[target_topic]
    msg_type = get_message(msg_type_str)

    # 4. 循环读取消息
    data_list = []
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == target_topic:
            # 反序列化二进制数据为 Python 对象
            msg = deserialize_message(data, msg_type)
            
            # 💡 针对 /cmd_vel 话题提取线速度和角速度
            # 如果你换了其他话题，需要修改这里获取的字段属性
            data_list.append({
                'timestamp': timestamp,
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z
            })
            
    print(f"提取完成！共从 {target_topic} 中读取到 {len(data_list)} 条有效数据。")
    return data_list

if __name__ == '__main__':
    # 方案 B 的路径配置（包放在 scripts/bags/my_tb3_bag）
    BAG_PATH = './bags/my_tb3_bag' 
    TOPIC_NAME = '/cmd_vel'   # 可根据需要修改为你录制的话题名
    
    # 执行读取
    results = read_bag(BAG_PATH, TOPIC_NAME)
    
    if results:
        # 转换为 Pandas 数据框并导出为 CSV
        df = pd.DataFrame(results)
        output_csv = './cmd_vel_data.csv'
        df.to_csv(output_csv, index=False)
        print(f"数据已成功保存至: {output_csv}")