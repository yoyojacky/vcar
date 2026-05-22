#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派4B智能小车控制程序 - 带超声波避障版本
新增：HC-SR04超声波传感器 + PC6开关控制 + 自动避障逻辑
"""

import RPi.GPIO as GPIO
import cv2
import time
import threading
import sys
import signal
import random
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Callable


# ==================== 配置区域 ====================

@dataclass
class PinConfig:
    """GPIO引脚配置"""
    
    # === L298N电机驱动引脚 ===
    IN1: int = 5        # 左电机方向1
    IN2: int = 6        # 左电机方向2
    IN3: int = 13       # 右电机方向1
    IN4: int = 19       # 右电机方向2
    ENA: int = 20       # 左电机PWM调速
    ENB: int = 21       # 右电机PWM调速
    
    # === 语音模块触发引脚（5个指令引脚，上升沿触发）===
    VOICE_FORWARD: int = 17     # 前进 (PA_1)
    VOICE_STOP: int = 22       # 停止 (PA_4)
    VOICE_BACKWARD: int = 23    # 后退 (PC_4)
    VOICE_LEFT: int = 24       # 左转 (PB_5)
    VOICE_RIGHT: int = 25      # 右转 (PB_6)
    
    # === PA2 双边沿控制摄像头（默认低电平）===
    PA2_CAMERA: int = 27       # PA_2：上升沿开摄像头，下降沿关摄像头
    
    # === 超声波传感器引脚 ===
    PC6_ULTRASONIC_EN: int = 26    # PC6：高电平打开超声波，低电平关闭
    ULTRASONIC_TRIG: int = 16      # HC-SR04 Trig（触发）
    ULTRASONIC_ECHO: int = 12      # HC-SR04 Echo（接收）
    
    @property
    def VOICE_PINS(self) -> list:
        return [
            self.VOICE_FORWARD, self.VOICE_STOP,
            self.VOICE_BACKWARD, self.VOICE_LEFT, self.VOICE_RIGHT
        ]


class MotorState(Enum):
    STOP = "停止"
    FORWARD = "前进"
    BACKWARD = "后退"
    LEFT = "左转"
    RIGHT = "右转"


class CameraState(Enum):
    CLOSED = "关闭"
    OPEN = "开启"
    ERROR = "错误"


# ==================== 电机控制类 ====================

class MotorController:
    """L298N双路电机驱动控制器"""
    
    def __init__(self, config: PinConfig):
        self.config = config
        self.pwm_a: Optional[GPIO.PWM] = None
        self.pwm_b: Optional[GPIO.PWM] = None
        self.current_speed = 80
        self.is_moving = False
        self._current_direction = MotorState.STOP
        self._lock = threading.Lock()
        self._setup_gpio()
    
    def _setup_gpio(self):
        motor_pins = [
            self.config.IN1, self.config.IN2,
            self.config.IN3, self.config.IN4,
            self.config.ENA, self.config.ENB
        ]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        self.pwm_a = GPIO.PWM(self.config.ENA, 1000)
        self.pwm_b = GPIO.PWM(self.config.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        print("[电机] GPIO初始化完成")
    
    def set_speed(self, speed: int):
        speed = max(0, min(100, speed))
        self.current_speed = speed
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)
        print(f"[电机] 速度: {speed}%")
    
    @property
    def current_direction(self) -> MotorState:
        return self._current_direction
    
    def move(self, direction: MotorState, duration: Optional[float] = None):
        with self._lock:
            self._stop_immediate()
            
            if direction == MotorState.STOP:
                self.is_moving = False
                self._current_direction = MotorState.STOP
                print("[电机] 停止")
                return
            
            self.is_moving = True
            self._current_direction = direction
            
            if direction == MotorState.FORWARD:
                GPIO.output(self.config.IN1, GPIO.HIGH)
                GPIO.output(self.config.IN2, GPIO.LOW)
                GPIO.output(self.config.IN3, GPIO.HIGH)
                GPIO.output(self.config.IN4, GPIO.LOW)
                print("[电机] ▶ 前进")
                
            elif direction == MotorState.BACKWARD:
                GPIO.output(self.config.IN1, GPIO.LOW)
                GPIO.output(self.config.IN2, GPIO.HIGH)
                GPIO.output(self.config.IN3, GPIO.LOW)
                GPIO.output(self.config.IN4, GPIO.HIGH)
                print("[电机] ◀ 后退")
                
            elif direction == MotorState.LEFT:
                GPIO.output(self.config.IN1, GPIO.LOW)
                GPIO.output(self.config.IN2, GPIO.HIGH)
                GPIO.output(self.config.IN3, GPIO.HIGH)
                GPIO.output(self.config.IN4, GPIO.LOW)
                print("[电机] ↺ 左转")
                
            elif direction == MotorState.RIGHT:
                GPIO.output(self.config.IN1, GPIO.HIGH)
                GPIO.output(self.config.IN2, GPIO.LOW)
                GPIO.output(self.config.IN3, GPIO.LOW)
                GPIO.output(self.config.IN4, GPIO.HIGH)
                print("[电机] ↻ 右转")
            
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
            
            if duration and duration > 0:
                threading.Timer(duration, self.stop).start()
    
    def _stop_immediate(self):
        GPIO.output(self.config.IN1, GPIO.LOW)
        GPIO.output(self.config.IN2, GPIO.LOW)
        GPIO.output(self.config.IN3, GPIO.LOW)
        GPIO.output(self.config.IN4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
    
    def stop(self):
        with self._lock:
            self._stop_immediate()
            self.is_moving = False
            self._current_direction = MotorState.STOP
            print("[电机] ◼ 已停止")
    
    def cleanup(self):
        self.stop()
        self.pwm_a.stop()
        self.pwm_b.stop()
        print("[电机] 资源已释放")


# ==================== 摄像头控制类 ====================

class CameraController:
    """OpenCV摄像头控制器"""
    
    def __init__(self, camera_index: int = 0):
        self.camera_index = camera_index
        self.cap: Optional[cv2.VideoCapture] = None
        self.state = CameraState.CLOSED
        self._lock = threading.Lock()
        self._frame_thread: Optional[threading.Thread] = None
        self._is_capturing = False
        self.latest_frame = None
    
    def open(self) -> bool:
        with self._lock:
            if self.state == CameraState.OPEN:
                return True
            
            try:
                self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                
                if not self.cap.isOpened():
                    raise RuntimeError("无法打开摄像头设备")
                
                for _ in range(5):
                    self.cap.read()
                
                self.state = CameraState.OPEN
                self._is_capturing = True
                self._frame_thread = threading.Thread(target=self._capture_loop, daemon=True)
                self._frame_thread.start()
                
                print(f"[摄像头] ✅ 已开启")
                return True
                
            except Exception as e:
                self.state = CameraState.ERROR
                print(f"[摄像头] ❌ 开启失败: {e}")
                return False
    
    def _capture_loop(self):
        while self._is_capturing and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.latest_frame = frame
            time.sleep(0.033)
    
    def close(self):
        with self._lock:
            self._is_capturing = False
            if self._frame_thread and self._frame_thread.is_alive():
                self._frame_thread.join(timeout=1.0)
            
            if self.cap:
                self.cap.release()
                self.cap = None
            
            self.state = CameraState.CLOSED
            self.latest_frame = None
            print("[摄像头] ✅ 已关闭")
    
    def take_photo(self, filename: Optional[str] = None) -> Optional[str]:
        with self._lock:
            if self.state != CameraState.OPEN or self.latest_frame is None:
                print("[摄像头] ❌ 错误：未开启或无可用帧")
                return None
            
            if filename is None:
                import os
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                save_dir = "/home/pi/photos"
                os.makedirs(save_dir, exist_ok=True)
                filename = f"{save_dir}/photo_{timestamp}.jpg"
            
            cv2.imwrite(filename, self.latest_frame)
            print(f"[摄像头] 📷 照片已保存: {filename}")
            return filename
    
    def get_status(self) -> str:
        return f"状态: {self.state.value}"


# ==================== 超声波传感器类 ====================

class UltrasonicSensor:
    """
    HC-SR04超声波距离传感器
    PC6控制开关，Trig触发，Echo接收
    """
    
    # 声速：343m/s @ 20°C，换算为 cm/us：34300 cm/s ÷ 1000000 = 0.0343 cm/us
    # 距离 = 时间(us) × 0.0343 / 2（往返）
    SPEED_OF_SOUND_CM_US = 0.01715  # cm per microsecond (half of 0.0343)
    
    def __init__(self, config: PinConfig):
        self.config = config
        self.is_enabled = False
        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_monitor = False
        self._last_distance = 999.0
        self._lock = threading.Lock()
        self._on_obstacle_callback: Optional[Callable] = None  # 避障回调
        
        self._setup_gpio()
    
    def _setup_gpio(self):
        """初始化超声波引脚"""
        # PC6：输出，控制超声波开关（默认低电平=关闭）
        GPIO.setup(self.config.PC6_ULTRASONIC_EN, GPIO.OUT)
        GPIO.output(self.config.PC6_ULTRASONIC_EN, GPIO.LOW)
        
        # Trig：输出，发送触发信号
        GPIO.setup(self.config.ULTRASONIC_TRIG, GPIO.OUT)
        GPIO.output(self.config.ULTRASONIC_TRIG, GPIO.LOW)
        
        # Echo：输入，接收回波
        GPIO.setup(self.config.ULTRASONIC_ECHO, GPIO.IN)
        
        print("[超声波] GPIO初始化完成 (Trig=GPIO16, Echo=GPIO12)")
    
    def enable(self):
        """打开超声波（PC6高电平）"""
        GPIO.output(self.config.PC6_ULTRASONIC_EN, GPIO.HIGH)
        self.is_enabled = True
        print("[超声波] ✅ 已启用 (PC6高电平)")
    
    def disable(self):
        """关闭超声波（PC6低电平）"""
        self._stop_monitor = True
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)
        
        GPIO.output(self.config.PC6_ULTRASONIC_EN, GPIO.LOW)
        self.is_enabled = False
        print("[超声波] ✅ 已禁用 (PC6低电平)")
    
    def measure_distance(self) -> float:
        """
        测量一次距离（cm）
        Returns:
            float: 距离厘米数，异常返回999.0
        """
        if not self.is_enabled:
            return 999.0
        
        with self._lock:
            # 确保Trig为低电平
            GPIO.output(self.config.ULTRASONIC_TRIG, GPIO.LOW)
            time.sleep(0.000002)  # 2us稳定
            
            # 发送10us的高电平触发信号
            GPIO.output(self.config.ULTRASONIC_TRIG, GPIO.HIGH)
            time.sleep(0.00001)   # 10us
            GPIO.output(self.config.ULTRASONIC_TRIG, GPIO.LOW)
            
            # 等待Echo变高（开始计时）
            start_time = time.time()
            timeout = start_time + 0.04  # 40ms超时
            
            while GPIO.input(self.config.ULTRASONIC_ECHO) == GPIO.LOW:
                start_time = time.time()
                if start_time > timeout:
                    return 999.0  # 超时
            
            # 等待Echo变低（结束计时）
            end_time = time.time()
            timeout = end_time + 0.04
            
            while GPIO.input(self.config.ULTRASONIC_ECHO) == GPIO.HIGH:
                end_time = time.time()
                if end_time > timeout:
                    return 999.0  # 超时
            
            # 计算距离
            duration = (end_time - start_time) * 1000000  # 转换为微秒
            distance = duration * self.SPEED_OF_SOUND_CM_US
            
            # 有效范围：2cm - 400cm
            if distance < 2 or distance > 400:
                return 999.0
            
            self._last_distance = distance
            return distance
    
    def get_distance(self) -> float:
        """获取最近一次测量的距离"""
        return self._last_distance
    
    def start_monitoring(self, interval: float = 0.1, threshold: float = 10.0):
        """
        启动后台持续监控线程
        Args:
            interval: 检测间隔（秒）
            threshold: 障碍物距离阈值（cm），小于此值触发避障
        """
        self._stop_monitor = False
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=(interval, threshold),
            daemon=True
        )
        self._monitor_thread.start()
        print(f"[超声波] 🔄 开始后台监控 (间隔{interval}s, 阈值{threshold}cm)")
    
    def _monitor_loop(self, interval: float, threshold: float):
        """后台监控循环"""
        while not self._stop_monitor and self.is_enabled:
            distance = self.measure_distance()
            
            if distance < threshold and self._on_obstacle_callback:
                print(f"\n[超声波] ⚠️  检测到障碍物! 距离: {distance:.1f}cm < {threshold}cm")
                self._on_obstacle_callback()
            
            time.sleep(interval)
    
    def register_obstacle_callback(self, callback: Callable):
        """注册避障回调函数"""
        self._on_obstacle_callback = callback
        print("[超声波] 已注册避障回调")
    
    def cleanup(self):
        """清理资源"""
        self.disable()
        print("[超声波] 资源已释放")


# ==================== 并行语音触发检测器 ====================

class ParallelVoiceDetector:
    """并行语音触发检测器"""
    
    def __init__(self, config: PinConfig):
        self.config = config
        self.is_listening = False
        self._callbacks: Dict[int, Callable] = {}
        self._last_trigger_time: Dict[int, int] = {}
        self._debounce_ms = 300
        
        self._pin_names = {
            config.VOICE_FORWARD: "前进",
            config.VOICE_STOP: "停止",
            config.VOICE_BACKWARD: "后退",
            config.VOICE_LEFT: "左转",
            config.VOICE_RIGHT: "右转",
            config.PA2_CAMERA: "摄像头(PA2)",
        }
        
        self._setup_gpio()
    
    def _setup_gpio(self):
        for pin in self.config.VOICE_PINS:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            self._last_trigger_time[pin] = 0
        
        GPIO.setup(self.config.PA2_CAMERA, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self._last_trigger_time[self.config.PA2_CAMERA] = 0
        
        print(f"[语音] 已配置 5个指令引脚 + PA2摄像头控制引脚")
    
    def register_callback(self, pin: int, callback: Callable):
        self._callbacks[pin] = callback
        name = self._pin_names.get(pin, f"引脚{pin}")
        print(f"[语音] 已注册 [{name}] GPIO{pin} 的回调")
    
    def _create_rising_callback(self, pin: int):
        def callback(channel):
            current_time = int(time.time() * 1000)
            if current_time - self._last_trigger_time[pin] < self._debounce_ms:
                return
            self._last_trigger_time[pin] = current_time
            
            if GPIO.input(pin) == GPIO.HIGH:
                name = self._pin_names.get(pin, f"引脚{pin}")
                print(f"\n[语音] 🔊 检测到: [{name}] GPIO{pin} 上升沿")
                if pin in self._callbacks:
                    try:
                        self._callbacks[pin]()
                    except Exception as e:
                        print(f"[语音] ❌ 回调错误: {e}")
        return callback
    
    def _create_both_callback(self, pin: int):
        def callback(channel):
            current_time = int(time.time() * 1000)
            if current_time - self._last_trigger_time[pin] < self._debounce_ms:
                return
            self._last_trigger_time[pin] = current_time
            
            level = GPIO.input(pin)
            name = self._pin_names.get(pin, f"引脚{pin}")
            
            if level == GPIO.HIGH:
                print(f"\n[语音] 📷 检测到: [{name}] 上升沿 → 打开摄像头")
                action = "open"
            else:
                print(f"\n[语音] 📷 检测到: [{name}] 下降沿 → 关闭摄像头")
                action = "close"
            
            if pin in self._callbacks:
                try:
                    self._callbacks[pin](action)
                except Exception as e:
                    print(f"[语音] ❌ 回调错误: {e}")
        return callback
    
    def start_listening(self):
        self.is_listening = True
        
        for pin in self.config.VOICE_PINS:
            GPIO.add_event_detect(
                pin,
                GPIO.RISING,
                callback=self._create_rising_callback(pin),
                bouncetime=50
            )
        
        GPIO.add_event_detect(
            self.config.PA2_CAMERA,
            GPIO.BOTH,
            callback=self._create_both_callback(self.config.PA2_CAMERA),
            bouncetime=50
        )
        
        print("[语音] 🎙️  开始监听...")
    
    def stop_listening(self):
        self.is_listening = False
        for pin in self.config.VOICE_PINS:
            try:
                GPIO.remove_event_detect(pin)
            except:
                pass
        try:
            GPIO.remove_event_detect(self.config.PA2_CAMERA)
        except:
            pass
        print("[语音] 🛑 已停止监听")
    
    def manual_trigger(self, pin: int, action: str = ""):
        if pin in self._callbacks:
            if action:
                self._callbacks[pin](action)
            else:
                self._callbacks[pin]()


# ==================== 主控制类 ====================

class SmartCarController:
    """智能小车主控制器（带超声波避障）"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self.config = PinConfig()
        self.motor = MotorController(self.config)
        self.camera = CameraController(camera_index=0)
        self.voice = ParallelVoiceDetector(self.config)
        self.ultrasonic = UltrasonicSensor(self.config)
        
        self.is_camera_on = False
        self.is_running = True
        self._is_avoiding = False  # 是否正在避障中
        
        # 注册避障回调
        self.ultrasonic.register_obstacle_callback(self._on_obstacle_detected)
        
        self._register_voice_callbacks()
        
        print("\n" + "="*50)
        print("    🚗 智能小车控制系统初始化完成")
        print("    ➕ 超声波避障功能已集成")
        print("="*50)
    
    def _register_voice_callbacks(self):
        """注册语音引脚回调"""
        self.voice.register_callback(self.config.VOICE_FORWARD, self._cmd_forward)
        self.voice.register_callback(self.config.VOICE_STOP, self._cmd_stop)
        self.voice.register_callback(self.config.VOICE_BACKWARD, self._cmd_backward)
        self.voice.register_callback(self.config.VOICE_LEFT, self._cmd_left)
        self.voice.register_callback(self.config.VOICE_RIGHT, self._cmd_right)
        self.voice.register_callback(self.config.PA2_CAMERA, self._cmd_camera_control)
    
    # ===== 避障逻辑 =====
    
    def _on_obstacle_detected(self):
        """
        超声波检测到障碍物时的回调
        停止前进，随机左转或右转
        """
        if self._is_avoiding:
            return  # 避免重复触发
        
        # 只有前进时才需要避障
        if self.motor.current_direction != MotorState.FORWARD:
            return
        
        self._is_avoiding = True
        print("\n" + "!"*40)
        print("[避障] 🚧 执行紧急避障程序!")
        print("!"*40)
        
        # 1. 立即停止
        self.motor.stop()
        time.sleep(0.3)
        
        # 2. 后退一点点（给自己留空间）
        self.motor.move(MotorState.BACKWARD)
        time.sleep(0.5)
        self.motor.stop()
        time.sleep(0.2)
        
        # 3. 随机转向（左转或右转）
        turn_direction = random.choice([MotorState.LEFT, MotorState.RIGHT])
        turn_name = "左转" if turn_direction == MotorState.LEFT else "右转"
        print(f"[避障] 🔄 随机选择: {turn_name}")
        
        self.motor.move(turn_direction)
        time.sleep(0.8)  # 转向时间
        self.motor.stop()
        time.sleep(0.2)
        
        # 4. 恢复前进
        print("[避障] ✅ 避障完成，恢复前进")
        self.motor.move(MotorState.FORWARD)
        
        self._is_avoiding = False
    
    # ===== 指令处理函数 =====
    
    def _cmd_forward(self):
        """前进 - 同时启用超声波"""
        self.ultrasonic.enable()
        self.ultrasonic.start_monitoring(interval=0.1, threshold=10.0)
        self.motor.move(MotorState.FORWARD)
    
    def _cmd_stop(self):
        """停止 - 同时关闭超声波"""
        self.ultrasonic.disable()
        self.motor.stop()
    
    def _cmd_backward(self):
        self.motor.move(MotorState.BACKWARD)
    
    def _cmd_left(self):
        self.motor.move(MotorState.LEFT)
    
    def _cmd_right(self):
        self.motor.move(MotorState.RIGHT)
    
    def _cmd_camera_control(self, action: str):
        if action == "open":
            if not self.is_camera_on:
                success = self.camera.open()
                self.is_camera_on = success
                if not success:
                    print("[错误] 摄像头开启失败")
            else:
                print("[提示] 摄像头已经是开启状态")
        elif action == "close":
            if self.is_camera_on:
                self.camera.close()
                self.is_camera_on = False
            else:
                print("[提示] 摄像头已经是关闭状态")
    
    # ===== 公共接口 =====
    
    def start(self):
        """启动语音监听主循环"""
        self.voice.start_listening()
        
        print("\n" + "-"*50)
        print("系统运行中，语音指令如下：")
        print("  [前进]     → GPIO17 上升沿（PA1）+ 自动启用超声波")
        print("  [停止]     → GPIO22 上升沿（PA4）+ 自动关闭超声波")
        print("  [后退]     → GPIO23 上升沿（PC4）")
        print("  [左转]     → GPIO24 上升沿（PB5）")
        print("  [右转]     → GPIO25 上升沿（PB6）")
        print("  [开摄像头] → GPIO27 上升沿（PA2）")
        print("  [关摄像头] → GPIO27 下降沿（PA2）")
        print("-"*50)
        print("超声波避障：前进时自动检测，<<10cm自动随机转向")
        print("按 Ctrl+C 退出程序\n")
        
        try:
            while self.is_running:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[系统] 检测到键盘中断")
        finally:
            self.shutdown()
    
    def manual_test(self, pin_name: str, action: str = ""):
        pin_map = {
            'forward': self.config.VOICE_FORWARD,
            'stop': self.config.VOICE_STOP,
            'backward': self.config.VOICE_BACKWARD,
            'left': self.config.VOICE_LEFT,
            'right': self.config.VOICE_RIGHT,
            'pa2': self.config.PA2_CAMERA,
        }
        
        pin = pin_map.get(pin_name.lower())
        if pin:
            print(f"\n[测试] 手动触发: {pin_name} {action}")
            self.voice.manual_trigger(pin, action)
        else:
            print(f"[测试] 未知命令: {pin_name}")
            print(f"可用: {list(pin_map.keys())}")
    
    def get_status(self) -> str:
        return f"""
========== 系统状态 ==========
电机: {'运行中' if self.motor.is_moving else '停止'} | 方向: {self.motor.current_direction.value}
速度: {self.motor.current_speed}%
摄像头: {self.camera.get_status()}
超声波: {'启用' if self.ultrasonic.is_enabled else '禁用'} | 距离: {self.ultrasonic.get_distance():.1f}cm
语音监听: {'运行中' if self.voice.is_listening else '停止'}
==============================
"""
    
    def shutdown(self):
        print("\n" + "="*50)
        print("[系统] 正在安全关闭...")
        
        self.is_running = False
        self.ultrasonic.cleanup()
        self.motor.cleanup()
        
        if self.is_camera_on:
            self.camera.close()
        
        self.voice.stop_listening()
        GPIO.cleanup()
        
        print("[系统] ✅ 已安全关闭")
        print("="*50 + "\n")
        sys.exit(0)


# ==================== 测试与运行 ====================

def test_all_functions(car: SmartCarController):
    print("\n========== 开始硬件测试 ==========\n")
    
    print("[测试] 电机测试...")
    car.motor.set_speed(60)
    
    for direction in [MotorState.FORWARD, MotorState.BACKWARD, 
                      MotorState.LEFT, MotorState.RIGHT]:
        print(f"\n--> 测试: {direction.value}")
        car.motor.move(direction)
        time.sleep(1.5)
        car.motor.stop()
        time.sleep(0.5)
    
    print("\n[测试] 超声波测试...")
    car.ultrasonic.enable()
    for _ in range(5):
        dist = car.ultrasonic.measure_distance()
        print(f"  距离: {dist:.1f}cm")
        time.sleep(0.5)
    car.ultrasonic.disable()
    
    print("\n[测试] 摄像头测试...")
    car._cmd_camera_control("open")
    time.sleep(2)
    car.camera.take_photo()
    time.sleep(1)
    car._cmd_camera_control("close")
    
    print("\n========== 测试完成 ==========\n")


def interactive_test(car: SmartCarController):
    print("""
========== 交互测试模式 ==========
输入命令测试对应功能:
  f / forward   - 前进（启用超声波）
  b / backward  - 后退
  l / left      - 左转
  r / right     - 右转
  s / stop      - 停止（关闭超声波）
  o / cam_on    - 开摄像头
  p / cam_off   - 关摄像头
  t / photo     - 拍照
  u / ultrasonic - 单次测距
  q / quit      - 退出
==================================
""")
    
    while True:
        try:
            cmd = input("命令> ").strip().lower()
            if cmd in ['q', 'quit', 'exit']:
                break
            elif cmd in ['f', 'forward']:
                car.manual_test('forward')
            elif cmd in ['b', 'backward', 'back']:
                car.manual_test('backward')
            elif cmd in ['l', 'left']:
                car.manual_test('left')
            elif cmd in ['r', 'right']:
                car.manual_test('right')
            elif cmd in ['s', 'stop']:
                car.manual_test('stop')
            elif cmd in ['o', 'cam_on', 'on']:
                car.manual_test('pa2', 'open')
            elif cmd in ['p', 'cam_off', 'off']:
                car.manual_test('pa2', 'close')
            elif cmd in ['t', 'photo', 'pic']:
                car.camera.take_photo()
            elif cmd in ['u', 'ultrasonic']:
                car.ultrasonic.enable()
                dist = car.ultrasonic.measure_distance()
                print(f"距离: {dist:.1f}cm")
                car.ultrasonic.disable()
            else:
                print("未知命令")
        except KeyboardInterrupt:
            break
    
    car.shutdown()


def main():
    car = SmartCarController()
    
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "1"
    
    if mode == "1" or mode == "voice":
        car.start()
    elif mode == "2" or mode == "test":
        test_all_functions(car)
        car.shutdown()
    elif mode == "3" or mode == "interactive":
        interactive_test(car)
    else:
        print(f"未知模式: {mode}")
        print("用法: python3 smartcar.py [1|voice|2|test|3|interactive]")
        car.shutdown()


if __name__ == "__main__":
    def signal_handler(sig, frame):
        print("\n[信号] 收到中断信号")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    main()
