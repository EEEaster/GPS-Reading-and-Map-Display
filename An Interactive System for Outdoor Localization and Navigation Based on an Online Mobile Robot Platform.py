import math
import os
import time
import threading
import random
import sys
import tempfile
import csv

import re
import json
from datetime import datetime, timedelta
from typing import Optional, Tuple, List, Dict
from collections import deque

import pynmea2
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                            QWidget, QLabel, QTextEdit, QPushButton, QGroupBox,
                            QProgressBar, QMessageBox, QSplitter, QComboBox,
                            QFileDialog, QTabWidget, QListWidget, QListWidgetItem,
                            QScrollArea, QCheckBox, QLineEdit, QDialog, QDialogButtonBox)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer, QUrl, pyqtSlot, QObject
from PyQt5.QtGui import QFont
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel

# ========================================================================
# GPS数据结构和坐标转换
# ========================================================================

class GPSData:
    """GPS数据结构"""
    def __init__(self):
        self.utctime = ''
        self.uc8_time = ''  # UTC+8时间
        self.raw_lat = ''  # 原始纬度数据
        self.raw_lat_dir = ''  # 纬度方向
        self.raw_lon = ''  # 原始经度数据
        self.raw_lon_dir = ''  # 经度方向
        self.wgs84_lat = 0.0  # WGS84纬度
        self.wgs84_lon = 0.0  # WGS84经度
        self.bd09_lat = 0.0  # BD09纬度
        self.bd09_lon = 0.0  # BD09经度
        self.numSv = ''
        self.msl = ''
        self.cogt = ''
        self.cogm = ''
        self.sog = ''  # 地面速度（节）
        self.kph = ''  # 地面速度（公里/小时）
        self.last_update = 0
        self.lock = threading.Lock()

# 全局GPS数据实例
gps_data = GPSData()

class CoordinateConverter:
    """坐标转换类"""
    def __init__(self):
        pass
    
    def _transform_lat(self, x: float, y: float) -> float:
        """WGS84转GCJ02的纬度转换辅助函数"""
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
        return ret

    def _transform_lon(self, x: float, y: float) -> float:
        """WGS84转GCJ02的经度转换辅助函数"""
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
        return ret

    def wgs84_to_gcj02(self, lon: float, lat: float) -> Tuple[float, float]:
        """
        将WGS84坐标系转换为GCJ02坐标系（火星坐标系）
        """
        a = 6378245.0
        ee = 0.00669342162296594323
        
        if lon < 72.004 or lon > 137.8347 or lat < 0.8293 or lat > 55.8271:
            return lon, lat
        
        dlat = self._transform_lat(lon - 105.0, lat - 35.0)
        dlon = self._transform_lon(lon - 105.0, lat - 35.0)
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
        dlon = (dlon * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
        
        gcj_lat = lat + dlat
        gcj_lon = lon + dlon
        
        return gcj_lon, gcj_lat

    def gcj02_to_bd09(self, lon: float, lat: float) -> Tuple[float, float]:
        """
        将GCJ02坐标系（火星坐标系）转换为BD09坐标系（百度坐标系）
        """
        z = math.sqrt(lon * lon + lat * lat) + 0.00002 * math.sin(lat * math.pi * 3000.0 / 180.0)
        theta = math.atan2(lat, lon) + 0.000003 * math.cos(lon * math.pi * 3000.0 / 180.0)
        bd_lon = z * math.cos(theta) + 0.0065
        bd_lat = z * math.sin(theta) + 0.006
        return bd_lon, bd_lat

    def wgs84_to_bd09(self, lon: float, lat: float) -> Tuple[float, float]:
        """
        将WGS84坐标系转换为BD09坐标系
        """
        gcj_lon, gcj_lat = self.wgs84_to_gcj02(lon, lat)
        return self.gcj02_to_bd09(gcj_lon, gcj_lat)

# 创建坐标转换器实例
coord_converter = CoordinateConverter()

# 百度地图API密钥
BAIDU_AK = "**************"#使用自己申请的百度地图API

# ========================================================================
# 导航状态管理类
# ========================================================================

class NavigationManager:
    """导航状态管理器"""
    
    def __init__(self):
        self.is_active = False
        self.original_start_point = None  # 原始起点（用于退出导航后恢复）
        self.original_route_points = None  # 新增：原始路线点
        self.navigation_start_time = None
        self.last_route_update_time = 0
        self.route_update_interval = 3  # 路线更新间隔（秒）
        self.arrival_threshold = 10  # 到达终点的距离阈值（米）
        self.last_position = None
        self.should_stop_updates = False  # 新增：是否应该停止路线更新
        
    def start_navigation(self, start_point, end_point, route_points=None):
        """开始导航"""
        self.is_active = True
        self.original_start_point = start_point.copy() if start_point else None
        self.original_route_points = route_points.copy() if route_points else None  # 保存原始路线
        self.navigation_start_time = time.time()
        self.last_route_update_time = 0
        self.last_position = None
        self.should_stop_updates = False  # 重置停止更新标志
        
    def stop_navigation(self):
        """停止导航"""
        self.is_active = False
        self.original_start_point = None
        self.original_route_points = None
        self.navigation_start_time = None
        self.last_route_update_time = 0
        self.last_position = None
        self.should_stop_updates = False  # 重置停止更新标志
        
    def should_update_route(self, current_position):
        """检查是否需要更新路线"""
        if not self.is_active or self.should_stop_updates:  # 新增：检查是否应该停止更新
            return False
            
        current_time = time.time()
        
        # 检查时间间隔
        if current_time - self.last_route_update_time < self.route_update_interval:
            return False
            
        # 检查位置变化
        if self.last_position:
            distance = self.calculate_distance(
                self.last_position[0], self.last_position[1],
                current_position[0], current_position[1]
            )
            # 如果移动距离超过20米，则更新路线
            if distance < 20:
                return False
        
        self.last_position = current_position
        self.last_route_update_time = current_time
        return True
        
    def check_arrival(self, current_position, end_point):
        """检查是否到达终点附近"""
        if not self.is_active or not end_point:
            return False
            
        distance = self.calculate_distance(
            current_position[0], current_position[1],
            end_point['lat'], end_point['lng']
        )
        
        return distance <= self.arrival_threshold
        
    def stop_route_updates(self):
        """停止路线更新（新增方法）"""
        self.should_stop_updates = True
        
    def get_original_route(self):
        """获取原始路线（新增方法）"""
        return self.original_route_points
        
    @staticmethod
    def calculate_distance(lat1, lon1, lat2, lon2):
        """计算两个坐标点之间的距离（米）"""
        # 将十进制度数转化为弧度
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # 哈弗辛公式
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        r = 6371000  # 地球平均半径，单位为米
        return c * r

# ========================================================================
# 串口GPS数据读取和解析函数
# ========================================================================

def auto_detect_gps_port() -> Optional[str]:
    """
    自动检测GPS设备连接的串口
    
    Returns:
        检测到的串口名称，如"COM7"，如果未找到则返回None
    """
    print("正在自动检测GPS串口...")
    
    # 获取所有可用串口
    available_ports = list(serial.tools.list_ports.comports())
    
    if not available_ports:
        print("未找到任何可用串口")
        return None
    
    print(f"找到 {len(available_ports)} 个串口:")
    for port in available_ports:
        print(f"  - {port.device}: {port.description}")
    
    # 尝试检测GPS设备
    gps_indicators = ['GPS', 'ublox', 'UART', 'USB Serial', 'CP210', 'FTDI', 'Prolific']
    
    for port_info in available_ports:
        port_name = port_info.device
        description = port_info.description
        
        # 检查描述中是否包含GPS相关关键词
        for indicator in gps_indicators:
            if indicator.lower() in description.lower():
                print(f"检测到可能的GPS设备: {port_name} ({description})")
                return port_name
        
        # 如果描述中没有明确标识，尝试连接并验证GPS数据
        try:
            print(f"尝试连接 {port_name} 并验证GPS数据...")
            with serial.Serial(port_name, 9600, timeout=1) as test_ser:
                start_time = time.time()
                while time.time() - start_time < 3:  # 尝试3秒
                    if test_ser.in_waiting > 0:
                        line = test_ser.readline().decode('ascii', errors='ignore').strip()
                        if line and (line.startswith('$GNGGA') or line.startswith('$GNRMC') or 
                                    line.startswith('$GPGGA') or line.startswith('$GPRMC')):
                            print(f"在 {port_name} 上检测到GPS数据: {line[:20]}...")
                            return port_name
        except (serial.SerialException, OSError) as e:
            print(f"无法打开串口 {port_name}: {e}")
            continue
    
    print("未检测到GPS设备")
    return None

def convert_dms_to_dd(dms_str, direction):
    """
    将度分格式(DDMM.MMMM)转换为十进制度格式(DD.DDDD)
    """
    try:
        # 提取度和分
        if '.' in dms_str:
            deg_part = dms_str.split('.')[0]
            if len(deg_part) > 2:
                # 经度：DDDMM.MMMM
                degrees = float(deg_part[:-2])
                minutes = float(deg_part[-2:] + '.' + dms_str.split('.')[1])
            else:
                # 纬度：DDMM.MMMM
                degrees = float(deg_part[:-2])
                minutes = float(deg_part[-2:] + '.' + dms_str.split('.')[1])
        else:
            if len(dms_str) > 2:
                degrees = float(dms_str[:-2])
                minutes = float(dms_str[-2:])
            else:
                degrees = 0.0
                minutes = float(dms_str)
        
        # 计算十进制度
        decimal_degrees = degrees + minutes / 60.0
        
        # 根据方向调整符号
        if direction in ['S', 'W']:
            decimal_degrees = -decimal_degrees
            
        return decimal_degrees
    except Exception as e:
        print(f"坐标转换错误: {e}")
        return 0.0

def parse_gngga(data):
    """解析GNGGA数据"""
    try:
        fields = data.split(',')
        if len(fields) < 13:
            return False
            
        with gps_data.lock:
            gps_data.utctime = fields[1] if fields[1] else '000000.000'
            
            # 转换为UTC+8时间
            gps_data.utc8_time = format_utc8_time(gps_data.utctime)
            
            # 正确解析原始经纬度数据
            if fields[2] and fields[3] and fields[4] and fields[5]:
                # 原始纬度数据和方向
                gps_data.raw_lat = fields[2]  # 原始纬度数据 (DDMM.MMMM)
                gps_data.raw_lat_dir = fields[3]  # 纬度方向 (N/S)
                
                # 原始经度数据和方向
                gps_data.raw_lon = fields[4]  # 原始经度数据 (DDDMM.MMMM)
                gps_data.raw_lon_dir = fields[5]  # 经度方向 (E/W)
                
                # 转换为WGS84十进制坐标
                wgs84_lat = convert_dms_to_dd(fields[2], fields[3])
                wgs84_lon = convert_dms_to_dd(fields[4], fields[5])
                
                gps_data.wgs84_lat = wgs84_lat
                gps_data.wgs84_lon = wgs84_lon
                
                # 转换为BD09坐标系
                try:
                    bd09_lon, bd09_lat = coord_converter.wgs84_to_bd09(wgs84_lon, wgs84_lat)
                    gps_data.bd09_lat = bd09_lat
                    gps_data.bd09_lon = bd09_lon
                except Exception as e:
                    print(f"坐标转换错误: {e}")
                    gps_data.bd09_lat = 0.0
                    gps_data.bd09_lon = 0.0
                
            gps_data.numSv = fields[7] if len(fields) > 7 and fields[7] else '0'
            if len(fields) > 9 and fields[9]:
                gps_data.msl = fields[9]
            gps_data.last_update = time.time()
        return True
    except Exception as e:
        print(f"解析GNGGA错误: {e}")
        return False

def parse_gnvtg(data):
    """解析GNVTG数据"""
    try:
        fields = data.split(',')
        if len(fields) < 10:
            return False
            
        with gps_data.lock:
            if fields[1]:
                gps_data.cogt = fields[1] + 'T'
            if len(fields) > 3:
                if fields[3] == 'M':
                    gps_data.cogm = '0.00'
                    if len(fields) > 5 and fields[5]:
                        gps_data.sog = fields[5]
                    if len(fields) > 7 and fields[7]:
                        gps_data.kph = fields[7]
                else:
                    # 修正字段索引，确保正确解析速度数据
                    if fields[3]:
                        gps_data.cogm = fields[3]
                    # 确保字段索引正确，避免"A*16"这样的错误数据
                    if len(fields) > 7 and fields[7] and fields[7].replace('.', '').isdigit():
                        gps_data.sog = fields[7]
                    if len(fields) > 9 and fields[9] and fields[9].replace('.', '').isdigit():
                        gps_data.kph = fields[9]
            gps_data.last_update = time.time()
        return True
    except Exception as e:
        print(f"解析GNVTG错误: {e}")
        return False

def format_utc8_time(utc_time_str: str) -> str:
    """
    将UTC时间字符串转换为UTC+8时间格式
    UTC时间格式: hhmmss.sss -> UTC+8: hh:mm:ss
    """
    try:
        if not utc_time_str or len(utc_time_str) < 6:
            return "未知时间"
        
        # 提取小时、分钟、秒
        hours = int(utc_time_str[0:2])
        minutes = int(utc_time_str[2:4])
        seconds = int(float(utc_time_str[4:]))
        
        # 转换为UTC+8
        hours_utc8 = (hours + 8) % 24
        
        # 格式化为XX:XX:XX
        return f"{hours_utc8:02d}:{minutes:02d}:{seconds:02d}"
    except Exception as e:
        print(f"时间转换错误: {e}")
        return "时间错误"

# ========================================================================
# txt文件格式GPS信息读取和处理
# ========================================================================

class GPSProcessor:
    """GPS数据处理类"""
    
    def __init__(self):
        self.file_encoding = 'utf-8'
        self.last_file_size = 0
        self.current_original_coord = None
        self.current_bd09_coord = None
        self.baidu_ak = BAIDU_AK
        self.map_zoom_level = 16
        
    def detect_file_encoding(self, file_path: str) -> str:
        """检测文件编码"""
        encodings = ['gbk', 'gb2312', 'utf-8', 'latin-1', 'cp1252']
        
        for encoding in encodings:
            try:
                with open(file_path, 'r', encoding=encoding) as file:
                    for _ in range(10):
                        line = file.readline()
                        if not line:
                            break
                return encoding
            except UnicodeDecodeError:
                continue
            except Exception:
                continue
        
        return 'utf-8'
    
    def parse_nmea_with_timestamp(self, line: str) -> Optional[Tuple[float, float, float, str]]:
        """解析NMEA语句并添加时间戳"""
        try:
            msg = pynmea2.parse(line)
            
            if (hasattr(msg, 'latitude') and hasattr(msg, 'longitude') 
                and msg.latitude != 0.0 and msg.longitude != 0.0):
                
                if hasattr(msg, 'data_validity'):
                    if msg.data_validity != 'A':
                        return None
                
                current_timestamp = time.time()
                
                nmea_time = ""
                if hasattr(msg, 'timestamp') and msg.timestamp:
                    nmea_time = str(msg.timestamp)
                
                return (msg.latitude, msg.longitude, current_timestamp, nmea_time)
                
        except pynmea2.ParseError:
            return None
        except Exception:
            return None
        
        return None
    
    def format_local_time(self, timestamp: float) -> str:
        """将时间戳转换为本地时间字符串 (UTC+8)"""
        # 转换为UTC+8时区
        local_time = datetime.fromtimestamp(timestamp) + timedelta(hours=8)
        return local_time.strftime("%H:%M:%S UTC+8")
    
    def read_new_data(self, file_path: str) -> Optional[Tuple[float, float, float, str]]:
        """读取文件中的最新数据点"""
        latest_coord = None
        
        try:
            current_size = os.path.getsize(file_path)
            
            if current_size < self.last_file_size:
                self.last_file_size = 0
                print("检测到文件被重置，重新开始读取")
            
            if current_size > self.last_file_size:
                with open(file_path, 'r', encoding=self.file_encoding) as file:
                    file.seek(self.last_file_size)
                    new_lines = file.readlines()
                    
                    # 只处理最后几行，避免处理过多历史数据
                    for line in new_lines[-5:]:
                        line = line.strip()
                        if not line:
                            continue
                            
                        result = self.parse_nmea_with_timestamp(line)
                        if result:
                            latest_coord = result
                
                self.last_file_size = current_size
                
                if latest_coord:
                    lat, lon, timestamp, nmea_time = latest_coord
                    local_time = self.format_local_time(timestamp)
                    
            return latest_coord
            
        except Exception as e:
            print(f"读取新数据时出错: {e}")
            self.file_encoding = self.detect_file_encoding(file_path)
            return None
    
    def _transform_lat(self, x: float, y: float) -> float:
        """WGS84转GCJ02的纬度转换辅助函数"""
        ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(y * math.pi) + 40.0 * math.sin(y / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(y / 12.0 * math.pi) + 320 * math.sin(y * math.pi / 30.0)) * 2.0 / 3.0
        return ret

    def _transform_lon(self, x: float, y: float) -> float:
        """WGS84转GCJ02的经度转换辅助函数"""
        ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * math.sqrt(abs(x))
        ret += (20.0 * math.sin(6.0 * x * math.pi) + 20.0 * math.sin(2.0 * x * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(x * math.pi) + 40.0 * math.sin(x / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(x / 12.0 * math.pi) + 300.0 * math.sin(x / 30.0 * math.pi)) * 2.0 / 3.0
        return ret

    def wgs84_to_gcj02(self, lon: float, lat: float) -> Tuple[float, float]:
        """将WGS84坐标系转换为GCJ02坐标系（火星坐标系）"""
        a = 6378245.0
        ee = 0.00669342162296594323
        
        if lon < 72.004 or lon > 137.8347 or lat < 0.8293 or lat > 55.8271:
            return lon, lat
        
        dlat = self._transform_lat(lon - 105.0, lat - 35.0)
        dlon = self._transform_lon(lon - 105.0, lat - 35.0)
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
        dlon = (dlon * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
        
        gcj_lat = lat + dlat
        gcj_lon = lon + dlon
        
        return gcj_lon, gcj_lat

    def gcj02_to_bd09(self, lon: float, lat: float) -> Tuple[float, float]:
        """将GCJ02坐标系（火星坐标系）转换为BD09坐标系（百度坐标系）"""
        z = math.sqrt(lon * lon + lat * lat) + 0.00002 * math.sin(lat * math.pi * 3000.0 / 180.0)
        theta = math.atan2(lat, lon) + 0.000003 * math.cos(lon * math.pi * 3000.0 / 180.0)
        bd_lon = z * math.cos(theta) + 0.0065
        bd_lat = z * math.sin(theta) + 0.006
        return bd_lon, bd_lat

    def wgs84_to_bd09(self, lon: float, lat: float) -> Tuple[float, float]:
        """将WGS84坐标系转换为BD09坐标系"""
        gcj_lon, gcj_lat = self.wgs84_to_gcj02(lon, lat)
        return self.gcj02_to_bd09(gcj_lon, gcj_lat)

    def convert_coordinate(self, coord: Tuple[float, float, float, str]) -> Tuple[float, float, float, str]:
        """转换单个坐标从WGS84到BD09"""
        lat, lon, timestamp, nmea_time = coord
        bd_lon, bd_lat = self.wgs84_to_bd09(lon, lat)
        local_time_str = self.format_local_time(timestamp)
        return (bd_lat, bd_lon, timestamp, local_time_str)


# ========================================================================
# 串口GPS工作线程
# ========================================================================

class SerialGPSWorker(QThread):
    """串口GPS数据处理工作线程"""
    new_data = pyqtSignal(tuple)  # 发送新的坐标数据
    status_update = pyqtSignal(str)  # 发送状态更新
    error_occurred = pyqtSignal(str)  # 发送错误信息
    gps_data_updated = pyqtSignal()  # GPS数据更新信号
    navigation_update_needed = pyqtSignal(tuple)  # 新增：导航模式需要更新路线
    
    def __init__(self, port: str, baudrate: int = 9600, data_interval: float = 0.05):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.data_interval = data_interval
        self.is_running = False
        self.serial_port = None
        self.data_buffer = deque(maxlen=1024)
        
    def run(self):
        """线程主循环"""
        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.status_update.emit(f"串口连接成功: {self.port}")
        except Exception as e:
            self.error_occurred.emit(f"串口连接失败: {str(e)}")
            return
        
        self.is_running = True
        buffer = b''
        
        try:
            while self.is_running:
                # 读取串口数据
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer += data
                    
                    # 处理完整消息
                    while b'\r\n' in buffer:
                        line, buffer = buffer.split(b'\r\n', 1)
                        if line:
                            try:
                                decoded_line = line.decode('ascii', errors='ignore').strip()
                                if decoded_line:
                                    self.data_buffer.append(decoded_line)
                            except UnicodeDecodeError:
                                continue
                
                # 处理GPS数据
                if self.data_buffer:
                    line = self.data_buffer.popleft()
                    
                    # 解析GPS数据
                    if line.startswith('$GNGGA'):
                        if parse_gngga(line):
                            self.process_gps_data()
                    elif line.startswith('$GNVTG'):
                        parse_gnvtg(line)
                
                # 短暂休眠
                time.sleep(self.data_interval)
                
        except Exception as e:
            self.error_occurred.emit(f"数据处理错误: {str(e)}")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
    
    def process_gps_data(self):
        """处理GPS数据并发送信号"""
        with gps_data.lock:
            if gps_data.bd09_lat != 0.0 and gps_data.bd09_lon != 0.0:
                # 发送坐标数据
                coord = (gps_data.bd09_lat, gps_data.bd09_lon, gps_data.last_update, gps_data.utc8_time)
                self.new_data.emit(coord)
                self.gps_data_updated.emit()
                
                # 发送导航更新信号
                self.navigation_update_needed.emit(coord)
    
    def stop(self):
        """停止线程"""
        self.is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


class FileGPSWorker(QThread):
    """文件GPS数据处理工作线程（修改：添加导航模式支持）"""
    new_data = pyqtSignal(tuple)  # 发送新的坐标数据
    status_update = pyqtSignal(str)  # 发送状态更新
    error_occurred = pyqtSignal(str)  # 发送错误信息
    navigation_update_needed = pyqtSignal(tuple)  # 新增：导航模式需要更新路线
    
    def __init__(self, file_path: str, data_interval: float = 0.05):
        super().__init__()
        self.file_path = file_path
        self.data_interval = data_interval
        self.processor = GPSProcessor()
        self.is_running = False
        
    def run(self):
        """线程主循环"""
        if not os.path.exists(self.file_path):
            self.error_occurred.emit(f"文件不存在: {self.file_path}")
            return
        
        # 检测文件编码
        self.processor.file_encoding = self.processor.detect_file_encoding(self.file_path)
        self.status_update.emit(f"检测到文件编码: {self.processor.file_encoding}")
        
        # 初始化文件指针
        self.processor.last_file_size = os.path.getsize(self.file_path)
        self.is_running = True
        
        self.status_update.emit("开始实时监控GPS数据文件...")
        
        try:
            while self.is_running:
                # 读取最新数据
                new_coord = self.processor.read_new_data(self.file_path)
                
                if new_coord:
                    # 转换坐标
                    bd09_coord = self.processor.convert_coordinate(new_coord)
                    self.processor.current_original_coord = new_coord
                    self.processor.current_bd09_coord = bd09_coord
                    
                    # 发射新数据信号
                    self.new_data.emit(bd09_coord)
                    # 发射导航更新信号
                    self.navigation_update_needed.emit(bd09_coord)
                
                # 等待
                time.sleep(self.data_interval)
                
        except Exception as e:
            self.error_occurred.emit(f"数据处理错误: {str(e)}")
    
    def stop(self):
        """停止线程"""
        self.is_running = False


# ========================================================================
# 软件界面设计
# ========================================================================

class CommunicationHandler(QObject):
    """处理JavaScript与Python之间的通信"""
    
    # 定义信号
    start_point_set = pyqtSignal(float, float, str)
    end_point_set = pyqtSignal(float, float, str)
    route_planned = pyqtSignal(str, str, str, float, list)
    route_error = pyqtSignal(str)
    route_steps_ready = pyqtSignal(list)  # 新增：路线步骤信息
    search_results_ready = pyqtSignal(list)  # 新增：搜索结果
    navigation_started = pyqtSignal()  # 新增：导航开始
    navigation_ended = pyqtSignal()  # 新增：导航结束
    navigation_arrival_detected = pyqtSignal()  # 新增：检测到到达终点
    
    def __init__(self):
        super().__init__()
    
    @pyqtSlot(float, float, str)
    def onStartPointSet(self, lng, lat, address):
        """起点设置回调"""
        print(f"Start point set: {lng}, {lat}, {address}")
        self.start_point_set.emit(lng, lat, address)
    
    @pyqtSlot(float, float, str)
    def onEndPointSet(self, lng, lat, address):
        """终点设置回调"""
        print(f"End point set: {lng}, {lat}, {address}")
        self.end_point_set.emit(lng, lat, address)
    
    @pyqtSlot(str, str, str, float, list)
    def onRoutePlanned(self, route_type, distance, duration, taxi_fee, route_points):
        """路线规划完成回调"""
        print(f"Route planned: {route_type}, {distance}, {duration}")
        self.route_planned.emit(route_type, distance, duration, taxi_fee, route_points)
    
    @pyqtSlot(str)
    def onRouteError(self, route_type):
        """路线规划错误回调"""
        print(f"Route error: {route_type}")
        self.route_error.emit(route_type)
    
    @pyqtSlot(list)
    def onRouteStepsReady(self, steps):
        """路线步骤信息回调"""
        print(f"Route steps ready: {len(steps)} steps")
        self.route_steps_ready.emit(steps)
    
    @pyqtSlot(list)
    def onSearchResultsReady(self, results):
        """搜索结果回调"""
        print(f"Search results ready: {len(results)} results")
        self.search_results_ready.emit(results)
    
    @pyqtSlot()
    def onNavigationStarted(self):
        """导航开始回调"""
        print("Navigation started")
        self.navigation_started.emit()
    
    @pyqtSlot()
    def onNavigationEnded(self):
        """导航结束回调"""
        print("Navigation ended")
        self.navigation_ended.emit()
    
    @pyqtSlot()
    def onNavigationArrivalDetected(self):
        """检测到到达终点回调"""
        print("Navigation arrival detected")
        self.navigation_arrival_detected.emit()


class NavigationDialog(QDialog):
    """导航模式对话框"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("进入导航模式")
        self.setModal(True)
        self.setFixedSize(300, 200)
        
        layout = QVBoxLayout()
        
        # 提示信息
        label = QLabel("是否进入导航模式？\n\n在导航模式下，系统将：\n- 持续监控实时位置\n- 自动更新路线指引\n- 提供实时导航信息")
        layout.addWidget(label)
        
        # 按钮
        button_box = QDialogButtonBox(QDialogButtonBox.Yes | QDialogButtonBox.No)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)
        
        self.setLayout(layout)


class ArrivalDialog(QDialog):
    """到达目的地提示对话框"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("到达目的地")
        self.setModal(True)
        self.setFixedSize(350, 150)
        
        layout = QVBoxLayout()
        
        # 提示信息
        label = QLabel("已到达目的地附近！\n\n点击确定退出导航模式。")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)
        
        # 按钮
        button_box = QDialogButtonBox(QDialogButtonBox.Ok)
        button_box.accepted.connect(self.accept)
        layout.addWidget(button_box)
        
        self.setLayout(layout)


class SaveRouteDialog(QDialog):
    """保存路线对话框"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("保存路线")
        self.setModal(True)
        self.setFixedSize(350, 200)
        
        layout = QVBoxLayout()
        
        # 提示信息
        label = QLabel("是否保存当前路线到历史记录？\n\n保存后可以在历史轨迹中查看和管理。")
        layout.addWidget(label)
        
        # 路线名称输入
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("路线名称:"))
        self.route_name_input = QLineEdit()
        self.route_name_input.setPlaceholderText("请输入路线名称（可选）")
        name_layout.addWidget(self.route_name_input)
        layout.addLayout(name_layout)
        
        # 按钮
        button_box = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Discard | QDialogButtonBox.Cancel)
        button_box.button(QDialogButtonBox.Save).setText("保存")
        button_box.button(QDialogButtonBox.Discard).setText("不保存")
        button_box.button(QDialogButtonBox.Cancel).setText("取消")
        button_box.accepted.connect(self.accept_save)
        button_box.rejected.connect(self.accept_discard)
        button_box.clicked.connect(self.on_button_clicked)
        layout.addWidget(button_box)
        
        self.setLayout(layout)
        
        self.result = "cancel"  # 结果：save, discard, cancel
    
    def accept_save(self):
        """保存路线"""
        self.result = "save"
        self.accept()
    
    def accept_discard(self):
        """不保存路线"""
        self.result = "discard"
        self.accept()
    
    def on_button_clicked(self, button):
        """按钮点击处理"""
        if button.text() == "取消":
            self.result = "cancel"
            self.reject()
    
    def get_route_name(self):
        """获取路线名称"""
        name = self.route_name_input.text().strip()
        if not name:
            return f"路线_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        return name


class ClearMarkersDialog(QDialog):
    """清除标记对话框 - 带滚动条的版本"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("清除所有标记")
        self.setModal(True)
        self.setFixedSize(400, 300)  # 增加对话框尺寸
        
        layout = QVBoxLayout()
        
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        # 提示信息
        label = QLabel(
            "确定要清除地图上的所有标记吗？\n\n"
            "这将清除以下所有内容：\n"
            "• 起点和终点标记\n"
            "• 当前规划的路线\n" 
            "• 历史路线显示\n"
            "• 搜索结果标记\n"
            "• 所有其他地图覆盖物\n\n"
            "注意：此操作不可撤销，清除后需要重新设置起点和终点来规划路线。"
        )
        label.setWordWrap(True)
        label.setAlignment(Qt.AlignLeft)
        scroll_layout.addWidget(label)
        
        # 确认复选框
        self.confirm_checkbox = QCheckBox("我确认要清除所有标记，并了解此操作不可撤销")
        scroll_layout.addWidget(self.confirm_checkbox)
        
        scroll_area.setWidget(scroll_widget)
        layout.addWidget(scroll_area)
        
        # 按钮
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.accept)
        button_box.rejected.connect(self.reject)
        button_box.button(QDialogButtonBox.Ok).setEnabled(False)  # 初始禁用确定按钮
        
        # 连接复选框状态改变信号
        self.confirm_checkbox.stateChanged.connect(
            lambda state: button_box.button(QDialogButtonBox.Ok).setEnabled(state == Qt.Checked)
        )
        
        layout.addWidget(button_box)
        
        self.setLayout(layout)


class RouteStepsWindow(QMainWindow):
    """路线指引窗口"""
    def __init__(self, route_steps, parent=None):
        super().__init__(parent)
        self.route_steps = route_steps
        self.init_ui()
    
    def init_ui(self):
        self.setWindowTitle("路线指引")
        self.setGeometry(300, 300, 500, 600)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        # 标题
        title = QLabel(f"路线指引 - 共 {len(self.route_steps)} 个步骤")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(title)
        
        # 步骤显示区域
        scroll_area = QScrollArea()
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        for i, step in enumerate(self.route_steps):
            step_widget = self.create_step_widget(i+1, step)
            scroll_layout.addWidget(step_widget)
        
        scroll_layout.addStretch()
        scroll_area.setWidget(scroll_widget)
        layout.addWidget(scroll_area)
        
        # 关闭按钮
        close_btn = QPushButton("关闭")
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)
    
    def create_step_widget(self, step_num, step):
        """创建步骤显示部件"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        
        # 步骤编号
        num_label = QLabel(f"{step_num}")
        num_label.setFixedSize(30, 30)
        num_label.setAlignment(Qt.AlignCenter)
        num_label.setStyleSheet("""
            background-color: #4CAF50;
            color: white;
            border-radius: 15px;
            font-weight: bold;
        """)
        layout.addWidget(num_label)
        
        # 步骤信息
        info_widget = QWidget()
        info_layout = QVBoxLayout(info_widget)
        
        # 步骤描述
        desc_label = QLabel(step['description'])
        desc_label.setWordWrap(True)
        desc_label.setStyleSheet("font-weight: bold;")
        info_layout.addWidget(desc_label)
        
        # 步骤距离
        dist_label = QLabel(f"距离: {step['distance']}")
        dist_label.setStyleSheet("color: #666; font-size: 12px;")
        info_layout.addWidget(dist_label)
        
        layout.addWidget(info_widget)
        
        widget.setStyleSheet("""
            QWidget {
                border: 1px solid #ddd;
                border-radius: 5px;
                padding: 8px;
                margin: 2px;
                background-color: #f9f9f9;
            }
        """)
        
        return widget


class MainWindow(QMainWindow):
    """主窗口（修改：增强导航功能）"""
    
    def __init__(self):
        super().__init__()
        self.gps_worker = None
        self.serial_gps_worker = None
        self.baidu_ak = BAIDU_AK
        self.map_zoom_level = 16
        self.data_interval = 0.05
        self.map_html_file = None
        
        # 路线规划相关变量
        self.start_point = None
        self.end_point = None
        self.route_points = []
        self.current_route_type = "walking"
        self.route_steps = []
        self.current_real_time_coord = None  # 当前实时GPS坐标
        self.follow_realtime = True  # 是否跟随实时位置
        self.is_navigating = False  # 是否处于导航模式
        self.search_results = []  # 搜索结果
        self.search_history = []  # 搜索历史记录
        
        # 路径规划记忆功能相关变量
        self.route_history = []  # 存储历史路线记录
        self.current_history_route = None  # 当前显示的历史路线
        
        # 新增：导航管理器
        self.navigation_manager = NavigationManager()
        
        self.init_ui()
        self.setup_map()
        
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("在线移动机器人平台室外定位与导航的交互系统")
        self.setGeometry(100, 100, 1400, 900)
        
        # 设置应用样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #cccccc;
                border-radius: 5px;
                margin-top: 1ex;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                background-color: #4CAF50;
                border: none;
                color: white;
                padding: 8px 16px;
                text-align: center;
                text-decoration: none;
                font-size: 14px;
                margin: 4px 2px;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:disabled {
                background-color: #cccccc;
            }
            QPushButton#stopButton {
                background-color: #f44336;
            }
            QPushButton#stopButton:hover {
                background-color: #da190b;
            }
            QPushButton.active {
                background-color: #2196F3;
            }
            QPushButton.navigation {
                background-color: #FF9800;
            }
            QPushButton.navigation:hover {
                background-color: #F57C00;
            }
            QPushButton.clear {
                background-color: #9C27B0;
            }
            QPushButton.clear:hover {
                background-color: #7B1FA2;
            }
            QPushButton.history {
                background-color: #607D8B;
            }
            QPushButton.history:hover {
                background-color: #455A64;
            }
            QTextEdit {
                border: 1px solid #cccccc;
                border-radius: 4px;
                padding: 4px;
                background-color: white;
            }
            QLabel {
                padding: 2px;
            }
            QComboBox {
                padding: 5px;
                border: 1px solid #ccc;
                border-radius: 4px;
            }
            QListWidget {
                border: 1px solid #ccc;
                border-radius: 4px;
                padding: 5px;
                outline: none;
            }
            QListWidget::item {
                padding: 8px;
                border-bottom: 1px solid #eee;
            }
            QListWidget::item:selected {
                background-color: #e3f2fd;
            }
            QTabWidget::pane {
                border: 1px solid #C2C7CB;
                background-color: white;
            }
            QTabBar::tab {
                background-color: #E1E1E1;
                border: 1px solid #C4C4C3;
                padding: 8px 20px;
                margin-right: 2px;
            }
            QTabBar::tab:selected {
                background-color: #4CAF50;
                color: white;
            }
            QCheckBox {
                spacing: 5px;
            }
            QCheckBox::indicator {
                width: 15px;
                height: 15px;
            }
            QCheckBox::indicator:unchecked {
                border: 1px solid #ccc;
                background-color: white;
                border-radius: 3px;
            }
            QCheckBox::indicator:checked {
                border: 1px solid #4CAF50;
                background-color: #4CAF50;
                border-radius: 3px;
            }
            QLineEdit {
                padding: 5px;
                border: 1px solid #ccc;
                border-radius: 4px;
            }
        """)
        
        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # 创建主布局
        main_layout = QHBoxLayout(central_widget)
        
        # 创建分割器
        splitter = QSplitter(Qt.Horizontal)
        
        # 左侧控制面板
        left_panel = self.create_control_panel()
        splitter.addWidget(left_panel)
        
        # 右侧地图视图
        self.map_view = QWebEngineView()
        splitter.addWidget(self.map_view)
        
        # 设置分割器比例 - 调整比例使地图占比更大
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([350, 1050])  # 减小左侧面板宽度
        
        main_layout.addWidget(splitter)
        
    def create_control_panel(self):
        """创建控制面板"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # 标题
        title = QLabel("在线移动机器人平台室外定位与导航的交互系统")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("color: #2c3e50; margin: 10px;")
        layout.addWidget(title)
        
        # 创建选项卡
        self.tab_widget = QTabWidget()
        
        # 实时GPS追踪选项卡
        gps_tab = self.create_gps_tab()
        self.tab_widget.addTab(gps_tab, "实时GPS追踪")
        
        # 地点搜索选项卡
        search_tab = self.create_search_tab()
        self.tab_widget.addTab(search_tab, "地点搜索")
        
        # 路径规划选项卡
        route_tab = self.create_route_tab()
        self.tab_widget.addTab(route_tab, "路径规划")
        
        # 历史轨迹选项卡
        history_tab = self.create_history_tab()
        self.tab_widget.addTab(history_tab, "历史导航轨迹")
        
        layout.addWidget(self.tab_widget)
        
        return panel
    
    def create_gps_tab(self):
        """创建GPS追踪选项卡"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 数据源选择区域
        source_group = QGroupBox("数据源设置")
        source_layout = QVBoxLayout(source_group)
        
        # 数据源选择
        source_type_layout = QHBoxLayout()
        source_type_layout.addWidget(QLabel("数据源:"))
        self.source_combo = QComboBox()
        self.source_combo.addItems(["串口GPS设备", "GPS数据文件"])
        self.source_combo.currentTextChanged.connect(self.on_source_changed)
        source_type_layout.addWidget(self.source_combo)
        source_type_layout.addStretch()
        source_layout.addLayout(source_type_layout)
        
        # 串口设置（默认显示）
        self.serial_group = QGroupBox("串口设置")
        serial_layout = QVBoxLayout(self.serial_group)
        
        serial_btn_layout = QHBoxLayout()
        self.detect_serial_btn = QPushButton("自动检测串口")
        self.detect_serial_btn.clicked.connect(self.detect_serial_port)
        serial_btn_layout.addWidget(self.detect_serial_btn)
        
        serial_layout.addLayout(serial_btn_layout)
        
        self.serial_port_label = QLabel("未检测到串口")
        self.serial_port_label.setWordWrap(True)
        self.serial_port_label.setStyleSheet("background-color: #f8f9fa; padding: 8px; border-radius: 4px;")
        serial_layout.addWidget(self.serial_port_label)
        
        source_layout.addWidget(self.serial_group)
        
        # 文件设置（默认隐藏）
        self.file_group = QGroupBox("文件设置")
        file_layout = QVBoxLayout(self.file_group)
        
        file_btn_layout = QHBoxLayout()
        self.browse_btn = QPushButton("选择GPS数据文件")
        self.browse_btn.clicked.connect(self.browse_file)
        file_btn_layout.addWidget(self.browse_btn)
        
        file_layout.addLayout(file_btn_layout)
        
        self.file_path_label = QLabel("未选择文件")
        self.file_path_label.setWordWrap(True)
        self.file_path_label.setStyleSheet("background-color: #f8f9fa; padding: 8px; border-radius: 4px;")
        file_layout.addWidget(self.file_path_label)
        
        source_layout.addWidget(self.file_group)
        self.file_group.setVisible(False)  # 默认隐藏文件设置
        
        layout.addWidget(source_group)
        
        # 参数设置区域
        param_group = QGroupBox("参数设置")
        param_layout = QVBoxLayout(param_group)
        
        # 读取间隔设置
        interval_layout = QHBoxLayout()
        interval_layout.addWidget(QLabel("读取间隔:"))
        self.interval_combo = QComboBox()
        self.interval_combo.addItems(["0.05", "0.1", "0.2", "0.5", "1.0"])
        self.interval_combo.setCurrentText("0.05")
        interval_layout.addWidget(self.interval_combo)
        interval_layout.addWidget(QLabel("秒"))
        interval_layout.addStretch()
        param_layout.addLayout(interval_layout)
        
        # 地图级别设置
        zoom_layout = QHBoxLayout()
        zoom_layout.addWidget(QLabel("地图级别:"))
        self.zoom_combo = QComboBox()
        self.zoom_combo.addItems(["15", "16", "17", "18", "19"])
        self.zoom_combo.setCurrentText("16")
        self.zoom_combo.currentTextChanged.connect(self.on_zoom_changed)
        zoom_layout.addWidget(self.zoom_combo)
        zoom_layout.addStretch()
        param_layout.addLayout(zoom_layout)
        
        # 跟随实时位置设置
        follow_layout = QHBoxLayout()
        self.follow_checkbox = QCheckBox("地图跟随实时位置")
        self.follow_checkbox.setChecked(True)
        self.follow_checkbox.stateChanged.connect(self.on_follow_changed)
        follow_layout.addWidget(self.follow_checkbox)
        follow_layout.addStretch()
        param_layout.addLayout(follow_layout)
        
        layout.addWidget(param_group)
        
        # 状态信息区域
        status_group = QGroupBox("状态信息")
        status_layout = QVBoxLayout(status_group)
        
        # 连接状态
        status_indicator_layout = QHBoxLayout()
        self.status_indicator = QLabel("●")
        self.status_indicator.setStyleSheet("color: #e74c3c; font-size: 16px;")
        status_indicator_layout.addWidget(self.status_indicator)
        
        self.connection_status = QLabel("状态: 未连接")
        status_indicator_layout.addWidget(self.connection_status)
        status_indicator_layout.addStretch()
        status_layout.addLayout(status_indicator_layout)
        
        # 坐标信息
        coord_group = QGroupBox("坐标信息")
        coord_layout = QVBoxLayout(coord_group)
        
        self.coord_label = QLabel("纬度: --\n经度: --")
        self.coord_label.setStyleSheet("font-family: monospace; background-color: #f8f9fa; padding: 8px; border-radius: 4px;")
        coord_layout.addWidget(self.coord_label)
        
        self.time_label = QLabel("时间: --")
        coord_layout.addWidget(self.time_label)
        
        self.coord_system_label = QLabel("坐标系: BD09 (百度地图)")
        coord_layout.addWidget(self.coord_system_label)
        
        # GPS详细信息（串口模式显示）
        self.gps_detail_group = QGroupBox("GPS详细信息")
        gps_detail_layout = QVBoxLayout(self.gps_detail_group)
        
        self.satellite_label = QLabel("卫星数: --")
        gps_detail_layout.addWidget(self.satellite_label)
        
        self.altitude_label = QLabel("海拔: --")
        gps_detail_layout.addWidget(self.altitude_label)
        
        self.speed_label = QLabel("速度: --")
        gps_detail_layout.addWidget(self.speed_label)
        
        coord_layout.addWidget(self.gps_detail_group)
        self.gps_detail_group.setVisible(False)  # 默认隐藏详细信息
        
        status_layout.addWidget(coord_group)
        
        layout.addWidget(status_group)
        
        # 控制按钮区域
        control_group = QGroupBox("控制")
        control_layout = QVBoxLayout(control_group)
        
        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("开始监控")
        self.start_btn.clicked.connect(self.start_monitoring)
        btn_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("停止监控")
        self.stop_btn.setObjectName("stopButton")
        self.stop_btn.clicked.connect(self.stop_monitoring)
        self.stop_btn.setEnabled(False)
        btn_layout.addWidget(self.stop_btn)
        
        control_layout.addLayout(btn_layout)
        
        layout.addWidget(control_group)
        
        # 日志区域
        log_group = QGroupBox("运行日志")
        log_layout = QVBoxLayout(log_group)
        
        self.log_text = QTextEdit()
        self.log_text.setMaximumHeight(120)
        log_layout.addWidget(self.log_text)
        
        layout.addWidget(log_group)
        
        # 添加伸缩空间
        layout.addStretch()
        
        return tab
    
    def create_search_tab(self):
        """创建地点搜索选项卡"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 使用说明
        instruction_group = QGroupBox("使用说明")
        instruction_layout = QVBoxLayout()
        instruction_text = QTextEdit()
        instruction_text.setReadOnly(True)
        instruction_text.setPlainText(
            "1. 在搜索框中输入地点关键词\n"
            "2. 点击'搜索'按钮查找地点\n"
            "3. 在搜索结果列表中选择地点\n"
            "4. 选中的地点会自动设置为路径规划的终点\n"
            "5. 可以清除搜索结果或使用清除标记功能\n"
            "6. 快速搜索功能可以搜索实时位置附近的地点\n"
            "7. 使用'清除地图标点'按钮可清除地图上的搜索结果标记"
        )
        instruction_text.setMaximumHeight(140)
        instruction_layout.addWidget(instruction_text)
        instruction_group.setLayout(instruction_layout)
        layout.addWidget(instruction_group)
        
        # 搜索区域
        search_group = QGroupBox("地点搜索")
        search_layout = QVBoxLayout()
        
        # 搜索框
        search_input_layout = QHBoxLayout()
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("输入关键词（如：餐馆、医院、酒店...）")
        search_input_layout.addWidget(self.search_input)
        
        self.search_execute_btn = QPushButton("搜索")
        self.search_execute_btn.clicked.connect(self.on_execute_search)
        search_input_layout.addWidget(self.search_execute_btn)
        
        search_layout.addLayout(search_input_layout)
        
        # 快速搜索按钮
        quick_search_group = QGroupBox("快速搜索")
        quick_search_layout = QHBoxLayout()
        
        self.search_food_btn = QPushButton("美食")
        self.search_hotel_btn = QPushButton("酒店")
        self.search_scenic_btn = QPushButton("景点")
        self.search_station_btn = QPushButton("火车站")
        
        self.search_food_btn.clicked.connect(lambda: self.on_quick_search("美食"))
        self.search_hotel_btn.clicked.connect(lambda: self.on_quick_search("酒店"))
        self.search_scenic_btn.clicked.connect(lambda: self.on_quick_search("景点"))
        self.search_station_btn.clicked.connect(lambda: self.on_quick_search("火车站"))
        
        quick_search_layout.addWidget(self.search_food_btn)
        quick_search_layout.addWidget(self.search_hotel_btn)
        quick_search_layout.addWidget(self.search_scenic_btn)
        quick_search_layout.addWidget(self.search_station_btn)
        
        quick_search_group.setLayout(quick_search_layout)
        search_layout.addWidget(quick_search_group)
        
        # 操作按钮
        search_btn_layout = QHBoxLayout()
        self.clear_search_btn = QPushButton("清除搜索结果")
        self.clear_search_btn.clicked.connect(self.on_clear_search_results)
        search_btn_layout.addWidget(self.clear_search_btn)
        
        self.clear_search_markers_btn = QPushButton("清除地图标点")
        self.clear_search_markers_btn.clicked.connect(self.on_clear_search_markers)
        search_btn_layout.addWidget(self.clear_search_markers_btn)
        
        search_layout.addLayout(search_btn_layout)
        
        # 设为终点按钮
        use_as_end_layout = QHBoxLayout()
        self.use_as_end_btn = QPushButton("设为终点")
        self.use_as_end_btn.clicked.connect(self.on_use_search_as_end)
        use_as_end_layout.addWidget(self.use_as_end_btn)
        
        search_layout.addLayout(use_as_end_layout)
        
        search_group.setLayout(search_layout)
        layout.addWidget(search_group)
        
        # 搜索结果
        results_group = QGroupBox("搜索结果")
        results_layout = QVBoxLayout()
        
        self.search_results_list = QListWidget()
        self.search_results_list.itemClicked.connect(self.on_search_result_selected)
        results_layout.addWidget(self.search_results_list)
        
        results_group.setLayout(results_layout)
        layout.addWidget(results_group)
        
        # 搜索历史
        history_group = QGroupBox("搜索历史")
        history_layout = QVBoxLayout()
        
        self.search_history_list = QListWidget()
        self.search_history_list.itemClicked.connect(self.on_search_history_selected)
        history_layout.addWidget(self.search_history_list)
        
        # 历史操作按钮
        history_btn_layout = QHBoxLayout()
        self.clear_history_btn = QPushButton("清除历史")
        self.clear_history_btn.clicked.connect(self.on_clear_search_history)
        history_btn_layout.addWidget(self.clear_history_btn)
        
        history_layout.addLayout(history_btn_layout)
        history_group.setLayout(history_layout)
        layout.addWidget(history_group)
        
        # 添加弹性空间
        layout.addStretch()
        
        return tab
    
    def create_route_tab(self):
        """创建路径规划选项卡"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 使用说明
        instruction_group = QGroupBox("使用说明")
        instruction_layout = QVBoxLayout()
        instruction_text = QTextEdit()
        instruction_text.setReadOnly(True)
        instruction_text.setPlainText(
            "1. 设置起点和终点位置\n"
            "2. 点击'规划路线'生成路线\n"
            "3. 路线规划完成后可选择是否保存到历史记录\n"
            "4. 可以查看轨迹点坐标或导出为CSV文件\n"
            "5. 使用'路线指引'查看详细的导航步骤\n"
            "6. 选择是否进入导航模式\n"
        )
        instruction_text.setMaximumHeight(120)
        instruction_layout.addWidget(instruction_text)
        instruction_group.setLayout(instruction_layout)
        layout.addWidget(instruction_group)
        
        # 导航模式选择
        route_type_group = QGroupBox("导航模式")
        route_type_layout = QVBoxLayout()
        self.route_type_combo = QComboBox()
        self.route_type_combo.addItems(["步行导航", "骑行导航", "驾车导航"])
        self.route_type_combo.currentTextChanged.connect(self.on_route_type_changed)
        route_type_layout.addWidget(self.route_type_combo)
        route_type_group.setLayout(route_type_layout)
        layout.addWidget(route_type_group)
        
        # 起点设置
        start_group = QGroupBox("起点位置")
        start_layout = QVBoxLayout()
        self.start_coord_label = QLabel("经纬度：未设置")
        self.start_address_label = QLabel("地址：未设置")
        start_layout.addWidget(self.start_coord_label)
        start_layout.addWidget(self.start_address_label)
        
        # 起点按钮
        start_btn_layout = QHBoxLayout()
        self.set_start_btn = QPushButton("设置起点")
        self.set_start_btn.clicked.connect(self.on_set_start)
        start_btn_layout.addWidget(self.set_start_btn)
        
        self.use_realtime_btn = QPushButton("使用实时位置")
        self.use_realtime_btn.clicked.connect(self.on_use_realtime)
        self.use_realtime_btn.setEnabled(False)
        start_btn_layout.addWidget(self.use_realtime_btn)
        
        start_layout.addLayout(start_btn_layout)
        start_group.setLayout(start_layout)
        layout.addWidget(start_group)
        
        # 终点设置
        end_group = QGroupBox("终点位置")
        end_layout = QVBoxLayout()
        self.end_coord_label = QLabel("经纬度：未设置")
        self.end_address_label = QLabel("地址：未设置")
        end_layout.addWidget(self.end_coord_label)
        end_layout.addWidget(self.end_address_label)
        
        # 终点按钮
        end_btn_layout = QHBoxLayout()
        self.set_end_btn = QPushButton("设置终点")
        self.set_end_btn.clicked.connect(self.on_set_end)
        end_btn_layout.addWidget(self.set_end_btn)
        
        end_layout.addLayout(end_btn_layout)
        
        end_group.setLayout(end_layout)
        layout.addWidget(end_group)
        
        # 操作按钮
        button_group = QGroupBox("操作")
        button_layout = QVBoxLayout()
        
        # 第一行按钮
        btn_row1 = QHBoxLayout()
        self.route_btn = QPushButton("规划路线")
        self.clear_start_end_btn = QPushButton("清除起终点信息")
        
        self.route_btn.clicked.connect(self.on_route)
        self.clear_start_end_btn.clicked.connect(self.on_clear_start_end)
        
        self.route_btn.setEnabled(False)
        self.clear_start_end_btn.setEnabled(False)
        
        btn_row1.addWidget(self.route_btn)
        btn_row1.addWidget(self.clear_start_end_btn)
        button_layout.addLayout(btn_row1)
        
        # 第二行按钮
        btn_row2 = QHBoxLayout()
        self.show_coords_btn = QPushButton("显示轨迹点")
        self.export_btn = QPushButton("导出坐标")
        self.show_route_steps_btn = QPushButton("路线指引")  # 新增路线指引按钮
        
        self.show_coords_btn.clicked.connect(self.on_show_coords)
        self.export_btn.clicked.connect(self.on_export)
        self.show_route_steps_btn.clicked.connect(self.on_show_route_steps)  # 连接信号
        
        self.show_coords_btn.setEnabled(False)
        self.export_btn.setEnabled(False)
        self.show_route_steps_btn.setEnabled(False)  # 初始不可用
        
        btn_row2.addWidget(self.show_coords_btn)
        btn_row2.addWidget(self.export_btn)
        btn_row2.addWidget(self.show_route_steps_btn)
        button_layout.addLayout(btn_row2)
        
        # 第三行按钮 - 导航模式和清除标记
        btn_row3 = QHBoxLayout()
        self.navigation_btn = QPushButton("进入导航模式")
        self.clear_markers_btn = QPushButton("清除所有标记")
        
        self.navigation_btn.setObjectName("navigation")
        self.clear_markers_btn.setObjectName("clear")
        
        self.navigation_btn.clicked.connect(self.on_navigation)
        self.clear_markers_btn.clicked.connect(self.on_clear_markers)
        
        self.navigation_btn.setEnabled(False)
        
        btn_row3.addWidget(self.navigation_btn)
        btn_row3.addWidget(self.clear_markers_btn)
        button_layout.addLayout(btn_row3)
        
        # 第四行按钮 - 退出导航
        btn_row4 = QHBoxLayout()
        self.exit_navigation_btn = QPushButton("退出导航")
        self.exit_navigation_btn.setObjectName("stopButton")
        self.exit_navigation_btn.clicked.connect(self.on_exit_navigation)
        self.exit_navigation_btn.setEnabled(False)
        btn_row4.addWidget(self.exit_navigation_btn)
        btn_row4.addStretch()
        button_layout.addLayout(btn_row4)
        
        button_group.setLayout(button_layout)
        layout.addWidget(button_group)
        
        # 状态信息
        status_group = QGroupBox("状态信息")
        status_layout = QVBoxLayout()
        self.route_status_label = QLabel("请先设置起点和终点位置")
        self.route_status_label.setWordWrap(True)
        self.route_status_label.setMinimumHeight(50)  # 设置最小高度确保多行文本显示
        status_layout.addWidget(self.route_status_label)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 路线信息
        self.route_info_group = QGroupBox("路线信息")
        route_info_layout = QVBoxLayout()
        
        self.route_info_text = QTextEdit()
        self.route_info_text.setReadOnly(True)
        self.route_info_text.setMaximumHeight(120)
        route_info_layout.addWidget(self.route_info_text)
        
        self.route_info_group.setLayout(route_info_layout)
        layout.addWidget(self.route_info_group)
        
        # 添加弹性空间
        layout.addStretch()
        
        return tab
    
    def create_history_tab(self):
        """创建历史轨迹选项卡"""
        tab = QWidget()
        layout = QVBoxLayout(tab)
        
        # 使用说明
        instruction_group = QGroupBox("使用说明")
        instruction_layout = QVBoxLayout()
        instruction_text = QTextEdit()
        instruction_text.setReadOnly(True)
        instruction_text.setPlainText(
            "1. 查看已保存的历史路线记录\n"
            "2. 点击路线记录可以在地图上显示\n"
            "3. 可以删除不需要的历史路线\n"
            "4. 历史路线以紫色虚线显示\n"
            "5. 路线规划完成后会自动提示保存"
        )
        instruction_text.setMaximumHeight(100)
        instruction_layout.addWidget(instruction_text)
        instruction_group.setLayout(instruction_layout)
        layout.addWidget(instruction_group)
        
        # 历史记录操作
        operation_group = QGroupBox("操作")
        operation_layout = QVBoxLayout()
        
        # 操作按钮
        btn_layout = QHBoxLayout()
        self.show_history_btn = QPushButton("显示选中路线")
        self.hide_history_btn = QPushButton("隐藏历史路线")
        self.delete_history_btn = QPushButton("删除选中路线")
        
        self.show_history_btn.clicked.connect(self.on_show_history_route)
        self.hide_history_btn.clicked.connect(self.on_hide_history_route)
        self.delete_history_btn.clicked.connect(self.on_delete_history_route)
        
        btn_layout.addWidget(self.show_history_btn)
        btn_layout.addWidget(self.hide_history_btn)
        btn_layout.addWidget(self.delete_history_btn)
        
        operation_layout.addLayout(btn_layout)
        operation_group.setLayout(operation_layout)
        layout.addWidget(operation_group)
        
        # 历史记录列表
        history_list_group = QGroupBox("历史路线记录")
        history_list_layout = QVBoxLayout()
        
        self.history_list = QListWidget()
        self.history_list.itemClicked.connect(self.on_history_item_selected)
        history_list_layout.addWidget(self.history_list)
        
        history_list_group.setLayout(history_list_layout)
        layout.addWidget(history_list_group)
        
        # 历史路线详情
        history_detail_group = QGroupBox("路线详情")
        history_detail_layout = QVBoxLayout()
        
        self.history_detail_text = QTextEdit()
        self.history_detail_text.setReadOnly(True)
        self.history_detail_text.setMaximumHeight(150)
        history_detail_layout.addWidget(self.history_detail_text)
        
        history_detail_group.setLayout(history_detail_layout)
        layout.addWidget(history_detail_group)
        
        # 添加弹性空间
        layout.addStretch()
        
        return tab
    
    def setup_map(self):
        """设置百度地图"""
        # 生成HTML内容
        html_content = self.generate_map_html()
        
        # 创建临时HTML文件
        self.temp_html_file = tempfile.NamedTemporaryFile(
            mode='w', 
            suffix='.html', 
            encoding='utf-8', 
            delete=False
        )
        self.temp_html_file.write(html_content)
        self.temp_html_file.flush()
        
        # 加载地图
        url = QUrl.fromLocalFile(self.temp_html_file.name)
        self.map_view.load(url)
        
        # 创建通信处理器
        self.comm_handler = CommunicationHandler()
        
        # 创建WebChannel
        self.channel = QWebChannel()
        self.channel.registerObject('handler', self.comm_handler)
        self.map_view.page().setWebChannel(self.channel)
        
        # 连接信号
        self.comm_handler.start_point_set.connect(self.on_start_point_set)
        self.comm_handler.end_point_set.connect(self.on_end_point_set)
        self.comm_handler.route_planned.connect(self.on_route_planned)
        self.comm_handler.route_error.connect(self.on_route_error)
        self.comm_handler.route_steps_ready.connect(self.on_route_steps_ready)
        self.comm_handler.search_results_ready.connect(self.on_search_results_ready)
        self.comm_handler.navigation_started.connect(self.on_navigation_started)
        self.comm_handler.navigation_ended.connect(self.on_navigation_ended)
        self.comm_handler.navigation_arrival_detected.connect(self.on_navigation_arrival_detected)  # 新增
        
    def generate_map_html(self):
        """生成百度地图HTML内容（修改：增强导航功能）"""
        return f"""
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="initial-scale=1.0, user-scalable=no">
    <meta http-equiv="Content-Type" content="text/html; charset=utf-8">
    <title>在线移动机器人平台室外定位与导航的交互系统</title>
    <style>
        html, body, #container {{
            width: 100%; 
            height: 100%; 
            margin: 0; 
            padding: 0;
            font-family: "Microsoft YaHei", sans-serif;
        }}
        .info-panel {{
            position: absolute;
            top: 10px;
            left: 80px;
            background: white;
            padding: 12px;
            border-radius: 6px;
            box-shadow: 0 0 10px rgba(0,0,0,0.2);
            z-index: 1000;
            max-width: 300px;
            font-size: 12px;
        }}
        .timestamp {{
            color: #666;
            font-size: 11px;
            margin-top: 3px;
        }}
        .status {{
            color: #2ecc71;
            font-weight: bold;
        }}
        .navigation-status {{
            color: #FF9800;
            font-weight: bold;
        }}
        .coordinate {{
            font-family: monospace;
            background: #f5f5f5;
            padding: 4px 6px;
            border-radius: 3px;
            margin: 4px 0;
        }}
        .connection-status {{
            display: inline-block;
            width: 8px;
            height: 8px;
            border-radius: 50%;
            margin-right: 5px;
        }}
        .connected {{
            background-color: #2ecc71;
        }}
        .disconnected {{
            background-color: #e74c3c;
        }}
        .navigation-active {{
            background-color: #FF9800;
        }}
        .arrival-alert {{
            background-color: #4CAF50;
            color: white;
            padding: 10px;
            border-radius: 5px;
            margin-top: 5px;
            text-align: center;
            font-weight: bold;
        }}
    </style>
    <script type="text/javascript" src="https://api.map.baidu.com/api?v=3.0&ak={self.baidu_ak}"></script>
    <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
</head>
<body>
    <div class="info-panel">
        <h4 style="margin: 0 0 8px 0;">在线移动机器人平台室外定位与导航的交互系统</h4>
        <div id="navigationStatus" class="navigation-status" style="display: none;">
            <span class="connection-status navigation-active"></span>
            导航模式
        </div>
        <div class="status">
            <span id="connStatus" class="connection-status disconnected"></span>
            <span id="trackingStatus">未追踪</span>
        </div>
        <div class="coordinate" id="coordDisplay">
            纬度: --<br>经度: --
        </div>
        <div class="timestamp" id="timeDisplay">时间: --</div>
        <div class="timestamp">坐标系: BD09 (百度地图)</div>
        <div class="timestamp" id="followStatus">跟随模式: 开启</div>
        <div id="arrivalAlert" class="arrival-alert" style="display: none;">
            已到达目的地附近！
        </div>
    </div>
    <div id="container"></div>
    <script>
        // 初始化WebChannel
        var handler = null;
        new QWebChannel(qt.webChannelTransport, function(channel) {{
            handler = channel.objects.handler;
            console.log("WebChannel connected");
        }});
        
        // 创建地图实例
        var map = new BMap.Map("container");
        
        // 默认天安门为中心点坐标
        var defaultCenter = new BMap.Point(116.404, 39.915);
        
        // 初始化地图
        map.centerAndZoom(defaultCenter, {self.map_zoom_level});
        map.enableScrollWheelZoom(true);
        
        // 添加控件 - 移除三维地图选项
        map.addControl(new BMap.NavigationControl());
        map.addControl(new BMap.ScaleControl());
        map.addControl(new BMap.MapTypeControl({{
            mapTypes: [
                BMAP_NORMAL_MAP,
                BMAP_SATELLITE_MAP
            ]
        }}));
        
        // 全局变量
        var startMarker = null;
        var endMarker = null;
        var startPoint = null;
        var endPoint = null;
        var settingMode = null;
        var currentRouteType = 'walking';
        var routePoints = [];
        var routePolyline = null;
        var routeMarkers = []; // 存储路线标记
        var geocoder = new BMap.Geocoder();
        var localSearch = new BMap.LocalSearch(map, {{
            renderOptions: {{ map: map, autoViewport: false }},
            onSearchComplete: function(results) {{
                if (localSearch.getStatus() == BMAP_STATUS_SUCCESS) {{
                    var searchResults = [];
                    for (var i = 0; i < results.getCurrentNumPois(); i++) {{
                        var poi = results.getPoi(i);
                        searchResults.push({{
                            title: poi.title,
                            address: poi.address,
                            point: {{ lng: poi.point.lng, lat: poi.point.lat }}
                        }});
                    }}
                    // 通知Python
                    if (handler) {{
                        handler.onSearchResultsReady(searchResults);
                    }}
                }}
            }}
        }});
        var realtimeMarker = null;
        var realtimeLabel = null;
        var followRealtime = true; // 是否跟随实时位置
        var isNavigating = false; // 是否处于导航模式
        var searchMarkers = []; // 存储搜索结果的标记
        var historyPolyline = null; // 历史路线
        var originalStartPoint = null; // 原始起点（用于退出导航后恢复）
        var originalRoutePoints = null; // 新增：原始路线点
        var lastRouteUpdateTime = 0; // 上次路线更新时间
        var routeUpdateInterval = 5000; // 路线更新间隔（毫秒）
        var arrivalThreshold = 10; // 到达终点的距离阈值（米）
        var shouldStopRouteUpdates = false; // 新增：是否应该停止路线更新
        
        // 路线规划实例
        var walking = new BMap.WalkingRoute(map, {{
            renderOptions: {{
                map: map,
                autoViewport: true,
                selectFirstResult: true
            }}
        }});
        
        var biking = new BMap.RidingRoute(map, {{
            renderOptions: {{
                map: map,
                autoViewport: true,
                selectFirstResult: true
            }}
        }});
        
        var driving = new BMap.DrivingRoute(map, {{
            renderOptions: {{
                map: map,
                autoViewport: true,
                selectFirstResult: true
            }}
        }});
        
        // 清除所有覆盖物
        function clearAllOverlays() {{
            map.clearOverlays();
            
            // 重置所有变量
            startMarker = null;
            endMarker = null;
            startPoint = null;
            endPoint = null;
            routePolyline = null;
            routePoints = [];
            routeMarkers = [];
            historyPolyline = null;
            realtimeMarker = null;
            realtimeLabel = null;
            searchMarkers = [];
            originalStartPoint = null;
            originalRoutePoints = null;
            shouldStopRouteUpdates = false; // 重置停止更新标志
            
            // 清除搜索结果
            localSearch.clearResults();
            
            // 退出导航模式
            if (isNavigating) {{
                exitNavigation();
            }}
            
            console.log("已清除所有地图标记和覆盖物");
        }}
        
        // 清除路线标记
        function clearRouteMarkers() {{
            if (routeMarkers && routeMarkers.length > 0) {{
                for (var i = 0; i < routeMarkers.length; i++) {{
                    if (routeMarkers[i]) {{
                        map.removeOverlay(routeMarkers[i]);
                    }}
                }}
                routeMarkers = [];
            }}
        }}
        
        // 清除历史路线
        function clearHistoryRoute() {{
            if (historyPolyline) {{
                map.removeOverlay(historyPolyline);
                historyPolyline = null;
            }}
        }}
        
        // 清除搜索结果
        function clearSearchResults() {{
            localSearch.clearResults();
            // 清除搜索标记
            for (var i = 0; i < searchMarkers.length; i++) {{
                map.removeOverlay(searchMarkers[i]);
            }}
            searchMarkers = [];
        }}
        
        // 地图点击事件
        map.addEventListener("click", function(e) {{
            if (!settingMode) return;
            
            var point = e.point;
            
            if (settingMode === 'start') {{
                setStartPoint(point);
            }} else if (settingMode === 'end') {{
                setEndPoint(point);
            }}
            
            settingMode = null;
        }});
        
        // 设置起点
        function setStartPoint(point) {{
            startPoint = point;
            updateMarker('start', point);
            // 在导航模式下不进行逆编码
            if (!isNavigating) {{
                getAddress(point, function(address) {{
                    // 通知Python
                    if (handler) {{
                        handler.onStartPointSet(point.lng, point.lat, address);
                    }}
                }});
            }} else {{
                // 导航模式下直接使用坐标
                if (handler) {{
                    handler.onStartPointSet(point.lng, point.lat, '实时位置');
                }}
            }}
            checkRouteAvailability();
        }}
        
        // 设置终点
        function setEndPoint(point) {{
            endPoint = point;
            updateMarker('end', point);
            // 在导航模式下不进行逆编码
            if (!isNavigating) {{
                getAddress(point, function(address) {{
                    // 通知Python
                    if (handler) {{
                        handler.onEndPointSet(point.lng, point.lat, address);
                    }}
                }});
            }} else {{
                // 导航模式下直接使用坐标
                if (handler) {{
                    handler.onEndPointSet(point.lng, point.lat, '目的地');
                }}
            }}
            checkRouteAvailability();
        }}
        
        // 更新标记
        function updateMarker(type, point) {{
            if (type === 'start') {{
                if (startMarker) map.removeOverlay(startMarker);
                startMarker = new BMap.Marker(point);
                map.addOverlay(startMarker);
                var label = new BMap.Label("起点", {{offset: new BMap.Size(15, -10)}});
                startMarker.setLabel(label);
            }} else if (type === 'end') {{
                if (endMarker) map.removeOverlay(endMarker);
                endMarker = new BMap.Marker(point);
                map.addOverlay(endMarker);
                var label = new BMap.Label("终点", {{offset: new BMap.Size(15, -10)}});
                endMarker.setLabel(label);
            }}
            
            if (startPoint && endPoint) {{
                map.setViewport([startPoint, endPoint]);
            }}
        }}
        
        // 获取地址
        function getAddress(point, callback) {{
            geocoder.getLocation(point, function(result) {{
                if (result) {{
                    callback(result.address);
                }} else {{
                    callback("未知地址");
                }}
            }});
        }}
        
        // 检查是否可以规划路线
        function checkRouteAvailability() {{
            // 可以通过handler通知Python
        }}
        
        // 规划路线 - 主路线
        function planRoute() {{
            if (!startPoint || !endPoint) {{
                return;
            }}
            
            clearRoute();
            clearRouteMarkers();
            
            if (currentRouteType === 'walking') {{
                walking.search(startPoint, endPoint);
                walking.setSearchCompleteCallback(function(results) {{
                    handleRouteResults('walking', results);
                }});
            }} else if (currentRouteType === 'biking') {{
                biking.search(startPoint, endPoint);
                biking.setSearchCompleteCallback(function(results) {{
                    handleRouteResults('biking', results);
                }});
            }} else if (currentRouteType === 'driving') {{
                driving.search(startPoint, endPoint);
                driving.setSearchCompleteCallback(function(results) {{
                    handleRouteResults('driving', results);
                }});
            }}
        }}
        
        // 处理路线结果
        function handleRouteResults(type, results) {{
            var status = null;
            var plan = null;
            
            if (type === 'walking') status = walking.getStatus();
            else if (type === 'biking') status = biking.getStatus();
            else if (type === 'driving') status = driving.getStatus();
            
            if (status === BMAP_STATUS_SUCCESS) {{
                plan = results.getPlan(0);
                extractRoutePoints(plan);
                
                var distance = plan.getDistance(true);
                var duration = plan.getDuration(true);
                var taxiFee = type === 'driving' ? plan.getTaxiFee() || 0 : 0;
                
                // 提取路线步骤信息
                var steps = extractRouteSteps(plan);
                
                // 通知Python
                if (handler) {{
                    handler.onRoutePlanned(type, distance, duration, taxiFee, routePoints);
                    handler.onRouteStepsReady(steps);
                }}
            }} else {{
                // 通知Python
                if (handler) {{
                    handler.onRouteError(type);
                }}
            }}
        }}
        
        // 提取路线步骤信息
        function extractRouteSteps(plan) {{
            var steps = [];
            
            // 遍历所有路线段
            for (var i = 0; i < plan.getNumRoutes(); i++) {{
                var route = plan.getRoute(i);
                
                // 遍历路线段中的所有步骤
                for (var j = 0; j < route.getNumSteps(); j++) {{
                    var step = route.getStep(j);
                    steps.push({{
                        description: step.getDescription(true),  // 步骤描述（包含HTML格式）
                        distance: step.getDistance(true)         // 步骤距离
                    }});
                }}
            }}
            
            return steps;
        }}
        
        // 提取轨迹点
        function extractRoutePoints(plan) {{
            routePoints = [];
            
            for (var i = 0; i < plan.getNumRoutes(); i++) {{
                var route = plan.getRoute(i);
                var path = route.getPath();
                
                for (var j = 0; j < path.length; j++) {{
                    routePoints.push({{
                        lng: path[j].lng,
                        lat: path[j].lat,
                        index: routePoints.length
                    }});
                }}
            }}
            
            // 显示路线
            showRouteOnMap();
        }}
        
        // 显示路线
        function showRouteOnMap() {{
            if (routePolyline) {{
                map.removeOverlay(routePolyline);
            }}
            
            var points = [];
            for (var i = 0; i < routePoints.length; i++) {{
                points.push(new BMap.Point(routePoints[i].lng, routePoints[i].lat));
            }}
            
            var lineColor = "#FF0000";
            if (currentRouteType === 'biking') lineColor = "#00AA00";
            else if (currentRouteType === 'driving') lineColor = "#0000FF";
            
            routePolyline = new BMap.Polyline(points, {{
                strokeColor: lineColor,
                strokeWeight: 6,
                strokeOpacity: 0.8
            }});
            map.addOverlay(routePolyline);
        }}
        
        // 清除路线
        function clearRoute() {{
            if (routePolyline) {{
                map.removeOverlay(routePolyline);
                routePolyline = null;
            }}
            routePoints = [];
        }}
        
        // 显示历史路线
        function showHistoryRoute(routePoints, routeType) {{
            clearHistoryRoute();
            
            var points = [];
            for (var i = 0; i < routePoints.length; i++) {{
                points.push(new BMap.Point(routePoints[i].lng, routePoints[i].lat));
            }}
            
            var lineColor = "#9C27B0"; // 紫色表示历史路线
            historyPolyline = new BMap.Polyline(points, {{
                strokeColor: lineColor,
                strokeWeight: 4,
                strokeOpacity: 0.6,
                strokeStyle: "dashed"
            }});
            map.addOverlay(historyPolyline);
        }}
        
        // 清除所有标记
        function clearAllMarkers() {{
            // 清除路线
            clearRoute();
            clearHistoryRoute();
            clearRouteMarkers();
            
            // 清除起点和终点标记
            if (startMarker) {{
                map.removeOverlay(startMarker);
                startMarker = null;
            }}
            if (endMarker) {{
                map.removeOverlay(endMarker);
                endMarker = null;
            }}
            startPoint = null;
            endPoint = null;
            originalStartPoint = null;
            
            // 清除搜索结果
            clearSearchResults();
            
            // 退出导航模式
            if (isNavigating) {{
                exitNavigation();
            }}
        }}
        
        // 清除指定标记
        function clearMarkers(options) {{
            if (options.route) {{
                clearRoute();
                clearHistoryRoute();
                clearRouteMarkers();
            }}
            if (options.start) {{
                if (startMarker) {{
                    map.removeOverlay(startMarker);
                    startMarker = null;
                }}
                startPoint = null;
                originalStartPoint = null;
            }}
            if (options.end) {{
                if (endMarker) {{
                    map.removeOverlay(endMarker);
                    endMarker = null;
                }}
                endPoint = null;
            }}
            if (options.search) {{
                clearSearchResults();
            }}
            
            // 检查路线可用性
            checkRouteAvailability();
        }}
        
        // 设置模式
        function setMode(mode) {{
            settingMode = mode;
        }}
        
        // 设置路线类型
        function setRouteType(type) {{
            currentRouteType = type;
        }}
        
        // 获取轨迹点
        function getRoutePoints() {{
            return routePoints;
        }}
        
        // 更新实时位置
        function updateRealtimePosition(lon, lat, timeStr) {{
            // 清除之前的实时标记
            if (realtimeMarker) {{
                map.removeOverlay(realtimeMarker);
            }}
            if (realtimeLabel) {{
                map.removeOverlay(realtimeLabel);
            }}
            
            // 创建新标记 - 使用百度地图默认的红色标记图标
            var point = new BMap.Point(lon, lat);
            
            // 创建自定义图标 - 使用base64编码的红色指针图标
            var icon = new BMap.Icon('data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMzIiIGhlaWdodD0iMzIiIHZpZXdCb3g9IjAgMCAzMiAzMiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTE2IDJDMjEuNTIyOCAyIDI2IDYuNDc3MTUgMjYgMTJDMjYgMTcuNTIyOCAyMS41MjI4IDIyIDE2IDIyQzEwLjQ3NzIgMjIgNiAxNy41MjI4IDYgMTJDNiA2LjQ3NzE1IDEwLjQ3NzIgMiAxNiAyWk0xNiA1QzEyLjEzNjIgNSA5IDguMTM2MjUgOSAxMkM5IDE1Ljg2MzggMTIuMTM2MiAxOSAxNiAxOUMxOS44NjM4IDE5IDIzIDE1Ljg2MzggMjMgMTJDMjMgOC4xMzYyNSAxOS44NjM4IDUgMTYgNVoiIGZpbGw9IiNGRjQ0NDQiLz4KPHBhdGggZD0iTTE2IDhDMTcuNjU2OSA4IDE5IDkuMzQzMTUgMTkgMTFDMTkgMTIuNjU2OSAxNy42NTY5IDE0IDE2IDE0QzE0LjM0MzEgMTQgMTMgMTIuNjU2OSAxMyAxMUMxMyA5LjM0MzE1IDE0LjM0MzEgOCAxNiA4WiIgZmlsbD0iI0ZGNEREQyIvPgo8L3N2Zz4K', 
                new BMap.Size(32, 32), {{
                    anchor: new BMap.Size(16, 32),
                    imageSize: new BMap.Size(32, 32)
                }});
            
            realtimeMarker = new BMap.Marker(point, {{icon: icon}});
            
            // 创建时间标签
            realtimeLabel = new BMap.Label(timeStr, {{position: point}});
            realtimeLabel.setStyle({{
                color: "white",
                backgroundColor: "rgba(0,0,0,0.7)",
                border: "none",
                padding: "5px 8px",
                fontSize: "12px",
                borderRadius: "3px"
            }});
            
            // 添加到地图
            map.addOverlay(realtimeMarker);
            map.addOverlay(realtimeLabel);
            
            // 更新信息面板
            document.getElementById('coordDisplay').innerHTML = 
                '纬度: ' + lat.toFixed(6) + '<br>经度: ' + lon.toFixed(6);
            document.getElementById('timeDisplay').textContent = '时间: ' + timeStr;
            document.getElementById('connStatus').className = 'connection-status connected';
            document.getElementById('trackingStatus').textContent = '实时追踪中';
            
            // 如果设置了跟随模式，将地图中心移动到实时位置
            if (followRealtime) {{
                map.panTo(point);
            }}
            
            // 导航模式处理
            if (isNavigating) {{
                // 更新起点为当前位置
                setStartPoint(point);
                
                // 检查是否需要更新路线（避免频繁更新）
                var currentTime = new Date().getTime();
                if (currentTime - lastRouteUpdateTime > routeUpdateInterval && !shouldStopRouteUpdates) {{
                    planRoute();
                    lastRouteUpdateTime = currentTime;
                }}
                
                // 检查是否到达终点
                if (endPoint) {{
                    var distance = map.getDistance(point, endPoint);
                    if (distance <= arrivalThreshold) {{
                        showArrivalAlert();
                        shouldStopRouteUpdates = true; // 停止路线更新
                        if (handler) {{
                            handler.onNavigationArrivalDetected();
                        }}
                    }}
                }}
            }}
        }}
        
        // 显示到达提示
        function showArrivalAlert() {{
            document.getElementById('arrivalAlert').style.display = 'block';
            setTimeout(function() {{
                document.getElementById('arrivalAlert').style.display = 'none';
            }}, 5000); // 5秒后自动隐藏
        }}
        
        // 设置缩放级别
        function setZoomLevel(zoom) {{
            if (map) {{
                map.setZoom(zoom);
            }}
        }}
        
        // 设置跟随实时位置
        function setFollowRealtime(follow) {{
            followRealtime = follow;
            document.getElementById('followStatus').textContent = '跟随模式: ' + (follow ? '开启' : '关闭');
        }}
        
        // 搜索地点
        function searchPlaces(keyword) {{
            localSearch.clearResults();
            clearSearchResults(); // 清除之前的搜索标记
            localSearch.search(keyword);
        }}
        
        // 开始导航
        function startNavigation() {{
            if (!startPoint || !endPoint) {{
                alert("请先设置起点和终点");
                return;
            }}
            
            isNavigating = true;
            originalStartPoint = startPoint; // 保存原始起点
            originalRoutePoints = routePoints.slice(); // 保存原始路线点
            lastRouteUpdateTime = new Date().getTime();
            shouldStopRouteUpdates = false; // 重置停止更新标志
            document.getElementById('navigationStatus').style.display = 'block';
            document.getElementById('trackingStatus').textContent = '导航模式';
            
            // 通知Python
            if (handler) {{
                handler.onNavigationStarted();
            }}
        }}
        
        // 退出导航
        function exitNavigation() {{
            isNavigating = false;
            shouldStopRouteUpdates = false; // 重置停止更新标志
            document.getElementById('navigationStatus').style.display = 'none';
            document.getElementById('trackingStatus').textContent = '实时追踪中';
            
            // 恢复原始起点
            if (originalStartPoint) {{
                setStartPoint(originalStartPoint);
                originalStartPoint = null;
            }}
            
            // 恢复原始路线（新增：不再重新规划路线）
            if (originalRoutePoints) {{
                routePoints = originalRoutePoints.slice();
                showRouteOnMap();
                originalRoutePoints = null;
            }}
            
            // 通知Python
            if (handler) {{
                handler.onNavigationEnded();
            }}
        }}
        
        // 停止路线更新（新增函数）
        function stopRouteUpdates() {{
            shouldStopRouteUpdates = true;
        }}
        
        console.log("地图初始化完成");
        
        // 暴露函数给全局作用域
        window.setMode = setMode;
        window.planRoute = planRoute;
        window.clearAllMarkers = clearAllMarkers;
        window.clearMarkers = clearMarkers;
        window.setRouteType = setRouteType;
        window.getRoutePoints = getRoutePoints;
        window.updateRealtimePosition = updateRealtimePosition;
        window.setZoomLevel = setZoomLevel;
        window.setFollowRealtime = setFollowRealtime;
        window.searchPlaces = searchPlaces;
        window.startNavigation = startNavigation;
        window.exitNavigation = exitNavigation;
        window.clearRoute = clearRoute;
        window.showHistoryRoute = showHistoryRoute;
        window.clearHistoryRoute = clearHistoryRoute;
        window.clearSearchResults = clearSearchResults;
        window.clearRouteMarkers = clearRouteMarkers;
        window.clearAllOverlays = clearAllOverlays;
        window.stopRouteUpdates = stopRouteUpdates; // 新增
    </script>
</body>
</html>
"""
    
    # ========================================================================
    # 自动检测GPS串口
    # ========================================================================
    
    def on_source_changed(self, source_type):
        """数据源类型改变"""
        if source_type == "串口GPS设备":
            self.serial_group.setVisible(True)
            self.file_group.setVisible(False)
            self.gps_detail_group.setVisible(True)  # 显示GPS详细信息
        else:
            self.serial_group.setVisible(False)
            self.file_group.setVisible(True)
            self.gps_detail_group.setVisible(False)  # 隐藏GPS详细信息
    
    def detect_serial_port(self):
        """自动检测串口"""
        detected_port = auto_detect_gps_port()
        if detected_port:
            self.serial_port_label.setText(f"检测到串口: {detected_port}")
            self.current_serial_port = detected_port
            self.log_message(f"自动检测到串口: {detected_port}")
        else:
            self.serial_port_label.setText("未检测到GPS串口设备")
            self.current_serial_port = None
            self.log_message("未检测到可用的GPS串口设备")
    
    def browse_file(self):
        """选择文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "选择GPS数据文件", "", "Text Files (*.txt);;All Files (*)")
        if file_path:
            # 更新文件路径标签
            self.file_path_label.setText(os.path.basename(file_path))
            # 保存完整路径到属性中
            self.current_file_path = file_path
            self.log_message(f"已选择文件: {os.path.basename(file_path)}")
    
    def start_monitoring(self):
        """开始监控"""
        source_type = self.source_combo.currentText()
        
        if source_type == "串口GPS设备":
            self.start_serial_monitoring()
        else:
            self.start_file_monitoring()
    
    def start_serial_monitoring(self):
        """开始串口监控（修改：连接导航更新信号）"""
        if not hasattr(self, 'current_serial_port') or not self.current_serial_port:
            QMessageBox.warning(self, "警告", "请先检测并选择串口")
            return
        
        # 更新参数
        self.data_interval = float(self.interval_combo.currentText())
        self.map_zoom_level = int(self.zoom_combo.currentText())
        
        # 创建并启动串口工作线程
        self.serial_gps_worker = SerialGPSWorker(
            self.current_serial_port, 
            9600, 
            self.data_interval
        )
        self.serial_gps_worker.new_data.connect(self.on_new_data)
        self.serial_gps_worker.status_update.connect(self.on_status_update)
        self.serial_gps_worker.error_occurred.connect(self.on_error)
        self.serial_gps_worker.gps_data_updated.connect(self.on_gps_data_updated)
        self.serial_gps_worker.navigation_update_needed.connect(self.on_navigation_update_needed)  # 新增
        
        self.serial_gps_worker.start()
        
        # 更新UI状态
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.connection_status.setText("状态: 串口监控中...")
        self.status_indicator.setStyleSheet("color: #2ecc71; font-size: 16px;")
        self.use_realtime_btn.setEnabled(True)  # 启用使用实时位置按钮
        
        self.log_message(f"开始串口GPS监控: {self.current_serial_port}")
    
    def start_file_monitoring(self):
        """开始文件监控（修改：连接导航更新信号）"""
        if not hasattr(self, 'current_file_path') or not self.current_file_path:
            QMessageBox.warning(self, "警告", "请选择GPS数据文件")
            return
        
        file_path = self.current_file_path
        if not os.path.exists(file_path):
            QMessageBox.critical(self, "错误", f"文件不存在: {file_path}")
            return
        
        # 更新参数
        self.data_interval = float(self.interval_combo.currentText())
        self.map_zoom_level = int(self.zoom_combo.currentText())
        
        # 创建并启动工作线程
        self.gps_worker = FileGPSWorker(file_path, self.data_interval)
        self.gps_worker.new_data.connect(self.on_new_data)
        self.gps_worker.status_update.connect(self.on_status_update)
        self.gps_worker.error_occurred.connect(self.on_error)
        self.gps_worker.navigation_update_needed.connect(self.on_navigation_update_needed)  # 新增
        
        self.gps_worker.start()
        
        # 更新UI状态
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.connection_status.setText("状态: 文件监控中...")
        self.status_indicator.setStyleSheet("color: #2ecc71; font-size: 16px;")
        self.use_realtime_btn.setEnabled(True)  # 启用使用实时位置按钮
        
        self.log_message("开始监控GPS数据文件")
    
    def stop_monitoring(self):
        """停止监控"""
        # 停止串口监控
        if self.serial_gps_worker and self.serial_gps_worker.isRunning():
            self.serial_gps_worker.stop()
            self.serial_gps_worker.wait(2000)  # 等待2秒
        
        # 停止文件监控
        if self.gps_worker and self.gps_worker.isRunning():
            self.gps_worker.stop()
            self.gps_worker.wait(2000)  # 等待2秒
        
        # 更新UI状态
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.connection_status.setText("状态: 已停止")
        self.status_indicator.setStyleSheet("color: #e74c3c; font-size: 16px;")
        self.use_realtime_btn.setEnabled(False)  # 禁用使用实时位置按钮
        
        self.log_message("停止监控GPS数据")
    
    def on_gps_data_updated(self):
        """GPS数据更新（串口模式）"""
        # 更新GPS详细信息
        with gps_data.lock:
            self.satellite_label.setText(f"卫星数: {gps_data.numSv}")
            self.altitude_label.setText(f"海拔: {gps_data.msl} m")
            if gps_data.kph:
                self.speed_label.setText(f"速度: {gps_data.kph} km/h")
            elif gps_data.sog:
                self.speed_label.setText(f"速度: {gps_data.sog} 节")
    
    def on_new_data(self, coord):
        """处理新的坐标数据"""
        lat, lon, timestamp, local_time = coord
        self.current_real_time_coord = (lat, lon, timestamp, local_time)
        
        # 更新状态标签
        self.coord_label.setText(f"纬度: {lat:.6f}\n经度: {lon:.6f}")
        self.time_label.setText(f"时间: {local_time}")
        
        # 在地图上更新标记
        js_code = f"updateRealtimePosition({lon}, {lat}, '{local_time}');"
        self.map_view.page().runJavaScript(js_code)
        
        # 记录日志（限制频率）
        if random.random() < 0.05:  # 5%概率记录
            self.log_message(f"位置更新: ({lat:.6f}, {lon:.6f})")
    
    def on_navigation_update_needed(self, coord):
        """导航模式需要更新路线（新增方法）"""
        if not self.navigation_manager.is_active:
            return
            
        lat, lon, timestamp, local_time = coord
        current_position = (lat, lon)
        
        # 检查是否需要更新路线
        if self.navigation_manager.should_update_route(current_position):
            # 更新起点为当前位置
            if self.start_point:
                self.start_point.update({
                    'lat': lat,
                    'lng': lon,
                    'address': f'实时位置 ({local_time})'
                })
                
                # 在地图上更新起点
                js_code = f"""
                var point = new BMap.Point({lon}, {lat});
                setStartPoint(point);
                """
                self.map_view.page().runJavaScript(js_code)
                
                # 重新规划路线
                self.on_route()
                self.log_message(f"导航路线已更新 - 当前位置: ({lat:.6f}, {lon:.6f})")
        
        # 检查是否到达终点
        if self.end_point and self.navigation_manager.check_arrival(current_position, self.end_point):
            self.handle_navigation_arrival()
    
    def handle_navigation_arrival(self):
        """处理到达终点（修改：停止路线更新）"""
        if not self.navigation_manager.is_active:
            return
            
        # 停止路线更新
        self.navigation_manager.stop_route_updates()
        self.map_view.page().runJavaScript("stopRouteUpdates()")
        
        # 显示到达提示对话框
        dialog = ArrivalDialog(self)
        dialog.exec_()
        
        # 退出导航模式
        self.on_exit_navigation()
        
        self.log_message("已到达目的地附近，导航自动结束")
    
    def on_status_update(self, message):
        """处理状态更新"""
        self.log_message(message)
    
    def on_error(self, error_message):
        """处理错误信息"""
        self.log_message(f"错误: {error_message}")
        QMessageBox.critical(self, "错误", error_message)
        
        # 停止监控
        self.stop_monitoring()
    
    def log_message(self, message):
        """添加日志消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        # 自动滚动到底部
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    # ========================================================================
    # 地图搜索功能
    # ========================================================================
    
    def on_zoom_changed(self, zoom_level):
        """地图缩放级别改变"""
        self.map_zoom_level = int(zoom_level)
        js_code = f"setZoomLevel({zoom_level});"
        self.map_view.page().runJavaScript(js_code)
        self.log_message(f"地图缩放级别已更改为: {zoom_level}级")
    
    def on_follow_changed(self, state):
        """跟随模式改变"""
        self.follow_realtime = (state == Qt.Checked)
        js_code = f"setFollowRealtime({str(self.follow_realtime).lower()});"
        self.map_view.page().runJavaScript(js_code)
        status = "开启" if self.follow_realtime else "关闭"
        self.log_message(f"地图跟随模式已{status}")
    
    # 搜索相关方法
    def on_quick_search(self, keyword):
        """快速搜索"""
        self.search_input.setText(keyword)
        self.on_execute_search()
    
    def on_execute_search(self):
        """执行搜索"""
        keyword = self.search_input.text().strip()
        if not keyword:
            QMessageBox.warning(self, "警告", "请输入搜索关键词")
            return
        
        # 获取当前地图中心点作为搜索中心
        if self.current_real_time_coord:
            lat, lon, timestamp, local_time = self.current_real_time_coord
            center_js = f"{lon}, {lat}"
        else:
            center_js = "116.404, 39.915"  # 默认北京中心
        
        # 执行搜索
        js_code = f"searchPlaces('{keyword}');"
        self.map_view.page().runJavaScript(js_code)
        
        # 添加到搜索历史
        self.add_to_search_history(keyword)
        
        self.log_message(f"执行地点搜索: {keyword}")
    
    def add_to_search_history(self, keyword):
        """添加到搜索历史"""
        # 检查是否已存在
        for item in self.search_history:
            if item['keyword'] == keyword:
                # 更新搜索时间
                item['timestamp'] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                self.update_search_history_display()
                return
        
        # 添加到历史记录
        history_item = {
            'keyword': keyword,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        self.search_history.append(history_item)
        
        # 保持历史记录数量限制
        if len(self.search_history) > 20:
            self.search_history.pop(0)
        
        self.update_search_history_display()
    
    def update_search_history_display(self):
        """更新搜索历史显示"""
        self.search_history_list.clear()
        for item in reversed(self.search_history):  # 最新的显示在最上面
            list_item = QListWidgetItem(f"{item['keyword']}\n{item['timestamp']}")
            self.search_history_list.addItem(list_item)
    
    def on_search_history_selected(self, item):
        """搜索历史项被选中"""
        text = item.text().split('\n')[0]  # 获取关键词部分
        self.search_input.setText(text)
        self.on_execute_search()
    
    def on_clear_search_history(self):
        """清除搜索历史"""
        self.search_history.clear()
        self.search_history_list.clear()
        self.log_message("已清除搜索历史")
    
    def on_clear_search_results(self):
        """清除搜索结果"""
        self.search_results = []
        self.search_results_list.clear()
        self.log_message("已清除搜索结果列表")
    
    def on_clear_search_markers(self):
        """清除地图上的搜索结果标记"""
        self.map_view.page().runJavaScript("clearSearchResults()")
        self.log_message("已清除地图上的搜索结果标记")
    
    def on_use_search_as_end(self):
        """将选中的搜索结果设为终点"""
        current_item = self.search_results_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "警告", "请先选择一个搜索结果")
            return
        
        index = self.search_results_list.row(current_item)
        if 0 <= index < len(self.search_results):
            result = self.search_results[index]
            
            # 在地图上设置终点
            js_code = f"""
            var point = new BMap.Point({result['point']['lng']}, {result['point']['lat']});
            setEndPoint(point);
            """
            self.map_view.page().runJavaScript(js_code)
            
            self.log_message(f"已将搜索结果设为终点: {result['title']}")
    
    def on_search_result_selected(self, item):
        """搜索结果被选中"""
        index = self.search_results_list.row(item)
        if 0 <= index < len(self.search_results):
            result = self.search_results[index]
            
            # 自动设置为终点
            js_code = f"""
            var point = new BMap.Point({result['point']['lng']}, {result['point']['lat']});
            setEndPoint(point);
            """
            self.map_view.page().runJavaScript(js_code)
            
            self.log_message(f"选择目的地: {result['title']}")
    
    # 路径规划相关方法
    def on_set_start(self):
        """设置起点模式"""
        self.map_view.page().runJavaScript("setMode('start')")
        self.route_status_label.setText("请在地图上点击选择起点位置")
    
    def on_set_end(self):
        """设置终点模式"""
        self.map_view.page().runJavaScript("setMode('end')")
        self.route_status_label.setText("请在地图上点击选择终点位置")
    
    def on_use_realtime(self):
        """使用实时位置作为起点（只获取当前这一秒的位置，不持续更新）"""
        if not self.current_real_time_coord:
            QMessageBox.warning(self, "警告", "没有可用的实时GPS位置数据")
            return
        
        lat, lon, timestamp, local_time = self.current_real_time_coord
        
        # 不再设置持续更新的标志，只使用当前坐标设置一次起点
        js_code = f"""
        var point = new BMap.Point({lon}, {lat});
        setStartPoint(point);
        """
        self.map_view.page().runJavaScript(js_code)
        
        self.route_status_label.setText("已将当前实时位置设为起点")
        self.log_message(f"使用实时位置作为起点: ({lat:.6f}, {lon:.6f})")
    
    def on_route(self):
        """规划路线"""
        route_type_map = {
            "步行导航": "walking",
            "骑行导航": "biking", 
            "驾车导航": "driving"
        }
        
        route_type = route_type_map.get(self.route_type_combo.currentText(), "walking")
        self.map_view.page().runJavaScript(f"setRouteType('{route_type}')")
        self.map_view.page().runJavaScript("planRoute()")
        self.route_status_label.setText(f"正在规划{self.route_type_combo.currentText()}...")
    
    def on_clear_start_end(self):
        """清除起终点信息"""
        self.start_point = None
        self.end_point = None
        
        # 清除地图上的起点和终点标记
        js_code = """
        if (startMarker) {
            map.removeOverlay(startMarker);
            startMarker = null;
        }
        if (endMarker) {
            map.removeOverlay(endMarker);
            endMarker = null;
        }
        startPoint = null;
        endPoint = null;
        """
        self.map_view.page().runJavaScript(js_code)
        
        # 更新UI显示
        self.update_route_coord_display()
        self.check_route_availability()
        
        # 禁用相关按钮
        self.route_btn.setEnabled(False)
        self.show_coords_btn.setEnabled(False)
        self.export_btn.setEnabled(False)
        self.navigation_btn.setEnabled(False)
        self.clear_start_end_btn.setEnabled(False)
        self.show_route_steps_btn.setEnabled(False)  # 禁用路线指引按钮
        
        self.route_status_label.setText("已清除起点和终点位置信息")
        self.log_message("已清除起点和终点位置信息")
    
    def on_clear_markers(self):
        """清除所有地图标记"""
        dialog = ClearMarkersDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            # 使用clearOverlays()一次性清除所有覆盖物
            self.map_view.page().runJavaScript("clearAllOverlays();")
            
            # 更新Python端状态
            self.start_point = None
            self.end_point = None
            self.route_points = []
            self.route_steps = []
            self.route_info_text.clear()
            self.search_results = []
            self.search_results_list.clear()
            
            # 更新UI显示
            self.update_route_coord_display()
            self.check_route_availability()
            
            # 禁用相关按钮
            self.show_coords_btn.setEnabled(False)
            self.export_btn.setEnabled(False)
            self.navigation_btn.setEnabled(False)
            self.show_route_steps_btn.setEnabled(False)
            
            # 如果处于导航模式，退出导航模式
            if self.is_navigating:
                self.map_view.page().runJavaScript("exitNavigation();")
                self.is_navigating = False
                self.navigation_btn.setEnabled(True)
                self.exit_navigation_btn.setEnabled(False)
            
            self.route_status_label.setText("已清除所有地图标记")
            self.log_message("已清除所有地图标记")
    
    def on_show_coords(self):
        """显示轨迹点"""
        if not self.route_points:
            QMessageBox.information(self, "提示", "请先规划路线")
            return
        
        # 在新窗口中显示坐标
        coords_window = CoordsWindow(self.route_points, self)
        coords_window.show()
    
    def on_export(self):
        """导出坐标"""
        if not self.route_points:
            QMessageBox.information(self, "提示", "请先规划路线")
            return
        
        file_path, _ = QFileDialog.getSaveFileName(
            self, "导出坐标", "route_coordinates.csv", "CSV文件 (*.csv)"
        )
        
        if file_path:
            try:
                with open(file_path, 'w', newline='', encoding='utf-8') as file:
                    writer = csv.writer(file)
                    writer.writerow(['序号', '经度', '纬度'])
                    for i, point in enumerate(self.route_points):
                        writer.writerow([i+1, point['lng'], point['lat']])
                
                QMessageBox.information(self, "成功", f"坐标已导出到: {file_path}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"导出失败: {str(e)}")
    
    def on_show_route_steps(self):
        """显示路线指引"""
        if not self.route_steps:
            QMessageBox.information(self, "提示", "暂无路线步骤信息")
            return
        
        # 创建并显示路线指引窗口
        steps_window = RouteStepsWindow(self.route_steps, self)
        steps_window.show()
    
    def on_navigation(self):
        """进入导航模式（修改：保存原始路线）"""
        if not self.route_points:
            QMessageBox.information(self, "提示", "请先规划路线")
            return
        
        # 显示确认对话框
        dialog = NavigationDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            # 开始导航
            self.map_view.page().runJavaScript("startNavigation()")
            self.navigation_manager.start_navigation(self.start_point, self.end_point, self.route_points)
            self.is_navigating = True
            self.navigation_btn.setEnabled(False)
            self.exit_navigation_btn.setEnabled(True)
            self.route_status_label.setText("已进入导航模式 - 路线将根据实时位置自动更新")
            self.log_message("已进入导航模式")
    
    def on_exit_navigation(self):
        """退出导航模式（修改：恢复原始路线，不重新规划）"""
        # 停止路线更新
        self.navigation_manager.stop_route_updates()
        self.map_view.page().runJavaScript("stopRouteUpdates()")
        
        # 退出导航
        self.map_view.page().runJavaScript("exitNavigation()")
        self.navigation_manager.stop_navigation()
        self.is_navigating = False
        self.navigation_btn.setEnabled(True)
        self.exit_navigation_btn.setEnabled(False)
        
        # 恢复原始路线（如果存在）
        original_route = self.navigation_manager.get_original_route()
        if original_route:
            self.route_points = original_route
            self.route_status_label.setText("已退出导航模式 - 恢复原始路线")
            self.log_message("已退出导航模式并恢复原始路线")
        else:
            self.route_status_label.setText("已退出导航模式")
            self.log_message("已退出导航模式")
    
    def on_route_type_changed(self, text):
        """导航模式改变"""
        route_type_map = {
            "步行导航": "walking",
            "骑行导航": "biking", 
            "驾车导航": "driving"
        }
        self.current_route_type = route_type_map.get(text, "walking")
        self.map_view.page().runJavaScript(f"setRouteType('{self.current_route_type}')")
        
        # 如果已经规划了路线，重新规划
        if self.start_point and self.end_point and self.route_points:
            self.on_route()
    
    def update_route_status(self, message):
        """更新路径规划状态信息"""
        self.route_status_label.setText(message)
    
    def update_route_coord_display(self):
        """更新路径规划坐标显示"""
        if self.start_point:
            self.start_coord_label.setText(
                f"经纬度：{self.start_point['lng']:.6f}, {self.start_point['lat']:.6f}"
            )
            self.start_address_label.setText(f"地址：{self.start_point.get('address', '未知')}")
        else:
            self.start_coord_label.setText("经纬度：未设置")
            self.start_address_label.setText("地址：未设置")
        
        if self.end_point:
            self.end_coord_label.setText(
                f"经纬度：{self.end_point['lng']:.6f}, {self.end_point['lat']:.6f}"
            )
            self.end_address_label.setText(f"地址：{self.end_point.get('address', '未知')}")
        else:
            self.end_coord_label.setText("经纬度：未设置")
            self.end_address_label.setText("地址：未设置")
    
    def check_route_availability(self):
        """检查是否可以规划路线"""
        if self.start_point and self.end_point:
            self.route_btn.setEnabled(True)
            self.clear_start_end_btn.setEnabled(True)
            self.route_status_label.setText("起点和终点已设置完成，可以规划路线")
        else:
            self.route_btn.setEnabled(False)
            self.clear_start_end_btn.setEnabled(False)
    
    # 通信回调方法
    def on_start_point_set(self, lng, lat, address):
        """起点设置回调"""
        self.start_point = {'lng': lng, 'lat': lat, 'address': address}
        self.update_route_coord_display()
        self.check_route_availability()
        self.route_status_label.setText("起点设置完成")
    
    def on_end_point_set(self, lng, lat, address):
        """终点设置回调"""
        self.end_point = {'lng': lng, 'lat': lat, 'address': address}
        self.update_route_coord_display()
        self.check_route_availability()
        self.route_status_label.setText("终点设置完成")
    
    def on_route_planned(self, route_type, distance, duration, taxi_fee, route_points):
        """路线规划完成回调（修改：导航模式下不询问保存）"""
        self.route_points = route_points
        
        # 更新路线信息
        route_type_name = {
            'walking': '步行',
            'biking': '骑行', 
            'driving': '驾车'
        }.get(route_type, '步行')
        
        info_text = f"路线类型: {route_type_name}\n距离: {distance}\n时间: {duration}\n"
        
        if route_type == 'driving' and taxi_fee > 0:
            info_text += f"打车费用: {taxi_fee}元\n"
        
        info_text += f"\n共提取 {len(route_points)} 个轨迹点"
        
        self.route_info_text.setPlainText(info_text)
        self.show_coords_btn.setEnabled(True)
        self.export_btn.setEnabled(True)
        self.navigation_btn.setEnabled(True)
        self.clear_start_end_btn.setEnabled(True)
        self.show_route_steps_btn.setEnabled(True)  # 启用路线指引按钮
        
        # 根据是否在导航模式显示不同的状态信息
        if self.navigation_manager.is_active:
            self.route_status_label.setText(f"导航路线已更新 - {route_type_name}路线")
        else:
            self.route_status_label.setText(f"{route_type_name}路线规划成功！")
        
        # 只有在非导航模式下才询问是否保存路线
        if not self.navigation_manager.is_active:
            # 弹出保存路线对话框
            self.ask_save_route(route_type_name, distance, duration)
        
        # 如果处于导航模式，启用退出导航按钮
        if self.navigation_manager.is_active:
            self.exit_navigation_btn.setEnabled(True)
    
    def ask_save_route(self, route_type, distance, duration):
        """询问是否保存路线"""
        dialog = SaveRouteDialog(self)
        result = dialog.exec_()
        
        if result == QDialog.Accepted:
            if dialog.result == "save":
                self.save_current_route(dialog.get_route_name(), route_type, distance, duration)
                self.route_status_label.setText("路线已保存到历史记录")
                self.log_message("路线已保存到历史记录")
            elif dialog.result == "discard":
                self.route_status_label.setText("路线规划完成（未保存）")
                self.log_message("路线规划完成（用户选择不保存）")
        else:
            self.route_status_label.setText("路线规划完成（取消保存）")
            self.log_message("路线规划完成（用户取消保存）")
    
    def save_current_route(self, route_name, route_type, distance, duration):
        """保存当前路线到历史记录"""
        if not self.route_points or not self.start_point or not self.end_point:
            return
        
        # 生成路线记录
        route_record = {
            'name': route_name,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'route_type': self.current_route_type,
            'start_point': self.start_point.copy(),
            'end_point': self.end_point.copy(),
            'route_points': self.route_points.copy(),
            'route_steps': self.route_steps.copy(),
            'distance': distance,
            'duration': duration
        }
        
        # 添加到历史记录
        self.route_history.append(route_record)
        
        # 更新历史记录显示
        self.update_history_list()
    
    def update_history_list(self):
        """更新历史记录列表"""
        self.history_list.clear()
        for i, record in enumerate(self.route_history):
            route_type_name = {
                'walking': '步行',
                'biking': '骑行',
                'driving': '驾车'
            }.get(record['route_type'], '未知')
            
            item_text = f"{record['name']}\n{record['timestamp']} - {route_type_name} - {record['distance']}"
            item = QListWidgetItem(item_text)
            item.setData(Qt.UserRole, i)  # 存储索引
            self.history_list.addItem(item)
    
    def on_history_item_selected(self, item):
        """历史记录项被选中"""
        index = item.data(Qt.UserRole)
        if 0 <= index < len(self.route_history):
            record = self.route_history[index]
            self.show_history_details(record)
    
    def show_history_details(self, record):
        """显示历史路线详情"""
        route_type_name = {
            'walking': '步行',
            'biking': '骑行',
            'driving': '驾车'
        }.get(record['route_type'], '未知')
        
        detail_text = f"路线名称: {record['name']}\n"
        detail_text += f"保存时间: {record['timestamp']}\n"
        detail_text += f"路线类型: {route_type_name}\n"
        detail_text += f"距离: {record['distance']}\n"
        detail_text += f"时间: {record['duration']}\n"
        detail_text += f"起点: {record['start_point'].get('address', '未知')}\n"
        detail_text += f"终点: {record['end_point'].get('address', '未知')}\n"
        detail_text += f"轨迹点数量: {len(record['route_points'])}"
        
        self.history_detail_text.setPlainText(detail_text)
    
    def on_show_history_route(self):
        """显示选中的历史路线"""
        current_item = self.history_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "警告", "请选择一条历史路线")
            return
        
        index = current_item.data(Qt.UserRole)
        if 0 <= index < len(self.route_history):
            record = self.route_history[index]
            self.show_historical_route(record)
    
    def show_historical_route(self, record):
        """在地图上显示历史路线"""
        # 将路线点转换为JavaScript数组
        points_js = []
        for point in record['route_points']:
            points_js.append(f"{{lng: {point['lng']}, lat: {point['lat']}}}")
        
        points_str = '[' + ','.join(points_js) + ']'
        
        # 调用JavaScript函数显示历史路线
        js_code = f"showHistoryRoute({points_str}, '{record['route_type']}');"
        self.map_view.page().runJavaScript(js_code)
        
        self.log_message(f"已显示历史路线: {record['name']}")
    
    def on_hide_history_route(self):
        """隐藏历史路线"""
        self.map_view.page().runJavaScript("clearHistoryRoute()")
        self.history_detail_text.clear()
        self.log_message("已隐藏历史路线")
    
    def on_delete_history_route(self):
        """删除选中的历史路线"""
        current_item = self.history_list.currentItem()
        if not current_item:
            QMessageBox.warning(self, "警告", "请选择一条要删除的历史路线")
            return
        
        index = current_item.data(Qt.UserRole)
        if 0 <= index < len(self.route_history):
            record = self.route_history[index]
            
            reply = QMessageBox.question(self, "确认删除", 
                                       f"确定要删除路线 '{record['name']}' 吗？",
                                       QMessageBox.Yes | QMessageBox.No)
            
            if reply == QMessageBox.Yes:
                # 如果正在显示这条路线，先隐藏
                if self.current_history_route == record:
                    self.on_hide_history_route()
                    self.current_history_route = None
                
                # 删除记录
                self.route_history.pop(index)
                
                # 更新列表
                self.update_history_list()
                self.history_detail_text.clear()
                
                self.log_message(f"已删除历史路线: {record['name']}")
    
    def on_route_steps_ready(self, steps):
        """路线步骤信息回调"""
        self.route_steps = steps
        # 如果路线指引按钮可用，确保它处于启用状态
        if self.route_points:
            self.show_route_steps_btn.setEnabled(True)
    
    def on_search_results_ready(self, results):
        """搜索结果回调"""
        self.search_results = results
        self.search_results_list.clear()
        
        for result in results:
            item = QListWidgetItem(f"{result['title']}\n{result['address']}")
            self.search_results_list.addItem(item)
        
        self.log_message(f"搜索完成，找到 {len(results)} 个结果")
    
    def on_navigation_started(self):
        """导航开始回调"""
        self.is_navigating = True
        self.navigation_btn.setEnabled(False)
        self.exit_navigation_btn.setEnabled(True)
        self.route_status_label.setText("导航模式已启动")
        self.log_message("导航模式已启动")
    
    def on_navigation_ended(self):
        """导航结束回调"""
        self.is_navigating = False
        self.navigation_btn.setEnabled(True)
        self.exit_navigation_btn.setEnabled(False)
        self.route_status_label.setText("导航模式已结束")
        self.log_message("导航模式已结束")
    
    def on_navigation_arrival_detected(self):
        """检测到到达终点回调"""
        # 当JavaScript检测到到达终点时调用此方法
        self.handle_navigation_arrival()
    
    def clean_html(self, html_text):
        """清理HTML标签，返回纯文本"""
        clean = re.compile('<.*?>')
        return re.sub(clean, '', html_text)
    
    def on_route_error(self, route_type):
        """路线规划错误回调"""
        route_type_name = {
            'walking': '步行',
            'biking': '骑行',
            'driving': '驾车'
        }.get(route_type, '步行')
        self.route_status_label.setText(f"{route_type_name}路线规划失败，请重试")
        QMessageBox.warning(self, "路线规划失败", f"{route_type_name}路线规划失败，请检查起点终点位置或网络连接")
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        self.stop_monitoring()
        # 清理临时文件
        try:
            if hasattr(self, 'temp_html_file'):
                os.unlink(self.temp_html_file.name)
        except:
            pass
        event.accept()


class CoordsWindow(QMainWindow):
    """坐标显示窗口"""
    def __init__(self, route_points, parent=None):
        super().__init__(parent)
        self.route_points = route_points
        self.init_ui()
    
    def init_ui(self):
        self.setWindowTitle("轨迹点坐标")
        self.setGeometry(200, 200, 500, 600)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        layout = QVBoxLayout(central_widget)
        
        # 标题
        title = QLabel(f"共 {len(self.route_points)} 个轨迹点")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Arial", 14, QFont.Bold))
        layout.addWidget(title)
        
        # 坐标显示区域
        scroll_area = QScrollArea()
        scroll_widget = QWidget()
        scroll_layout = QVBoxLayout(scroll_widget)
        
        for i, point in enumerate(self.route_points):
            coord_label = QLabel(f"点{i+1}: {point['lng']:.6f}, {point['lat']:.6f}")
            coord_label.setFont(QFont("Courier", 10))
            coord_label.setStyleSheet("padding: 5px; border-bottom: 1px solid #eee;")
            scroll_layout.addWidget(coord_label)
        
        scroll_layout.addStretch()
        scroll_area.setWidget(scroll_widget)
        layout.addWidget(scroll_area)
        
        # 关闭按钮
        close_btn = QPushButton("关闭")
        close_btn.clicked.connect(self.close)
        layout.addWidget(close_btn)


def main():
    """主函数"""
    # 检查必要的库
    try:
        import pynmea2
        from PyQt5 import QtWidgets, QtWebEngineWidgets
        import serial
    except ImportError as e:
        print(f"缺少必要的库: {e}")
        print("请使用以下命令安装:")
        print("pip install pynmea2 PyQt5 PyQtWebEngine pyserial")
        return
    
    # 创建应用
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle('Fusion')
    
    # 创建主窗口
    window = MainWindow()
    window.show()
    
    # 运行应用
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
