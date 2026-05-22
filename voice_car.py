#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派4B智能小车控制程序 - 修复版
修复：
1. 电机不动：修复PWM引脚初始化冲突
2. 窗口不关闭：重写窗口管理逻辑，确保可靠关闭
"""

import RPi.GPIO as GPIO
import cv2
import time
import threading
import sys
import signal
import random
import os
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Callable


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
    
    # === PA2 摄像头控制（GPIO27，高电平开窗口，低电平关窗口）===
    PA2_CAMERA: int = 27
    
    # === 超声波传感器引脚 ===
    PC6_ULTRASONIC_EN: int = 26    # PC6：高电平打开超声波，低电平关闭
    ULTRASONIC_TRIG: int = 16      # HC-SR04 Trig
    ULTRASONIC_ECHO: int = 12      # HC-SR04 Echo
    
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
    PREVIEWING = "预览中"
    ERROR = "错误"


# ==================== 电机控制类（修复版）====================

class MotorController:
    """
    L298N双路电机驱动控制器 - 修复版
    修复：PWM引脚初始化顺序，避免GPIO输出与PWM冲突
    """
    
    def __init__(self, config: PinConfig):
        self.config = config
        self.pwm_a = None
        self.pwm_b = None
        self.current_speed = 80
        self.is_moving = False
        self._current_direction = MotorState.STOP
        self._lock = threading.Lock()
        self._setup_gpio()
    
    def _setup_gpio(self):
        """
        修复：先设置方向引脚为输出，再单独配置PWM引脚
        避免ENA/ENB同时被设为GPIO.LOW和PWM导致的冲突
        """
        # 1. 设置方向控制引脚
        direction_pins = [
            self.config.IN1, self.config.IN2,
            self.config.IN3, self.config.IN4
        ]
        for pin in direction_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # 2. 设置PWM使能引脚（先设为输出低电平）
        GPIO.setup(self.config.ENA, GPIO.OUT)
        GPIO.output(self.config.ENA, GPIO.LOW)
        GPIO.setup(self.config.ENB, GPIO.OUT)
        GPIO.output(self.config.ENB, GPIO.LOW)
        
        # 3. 创建PWM对象（这会重新配置引脚为PWM模式）
        self.pwm_a = GPIO.PWM(self.config.ENA, 1000)
        self.pwm_b = GPIO.PWM(self.config.ENB, 1000)
        
        # 4. 启动PWM，初始占空比0（停止状态）
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        
        print("[电机] GPIO初始化完成")
        print(f"       方向引脚: IN1={self.config.IN1}, IN2={self.config.IN2}, "
              f"IN3={self.config.IN3}, IN4={self.config.IN4}")
        print(f"       PWM引脚: ENA={self.config.ENA}, ENB={self.config.ENB}")
    
    def set_speed(self, speed: int):
        speed = max(0, min(100, speed))
        self.current_speed = speed
        # 只在电机运动时更新PWM
        if self.is_moving:
            self.pwm_a.ChangeDutyCycle(speed)
            self.pwm_b.ChangeDutyCycle(speed)
        print(f"[电机] 速度设置为: {speed}%")
    
    @property
    def current_direction(self) -> MotorState:
        return self._current_direction
    
    def move(self, direction: MotorState, duration: Optional[float] = None):
        with self._lock:
            # 先停止当前运动
            self._stop_immediate()
            
            if direction == MotorState.STOP:
                self.is_moving = False
                self._current_direction = MotorState.STOP
                print("[电机] 停止")
                return
            
            self.is_moving = True
            self._current_direction = direction
            
            # 设置方向
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
                # 左轮反转，右轮正转（原地左转）
                GPIO.output(self.config.IN1, GPIO.LOW)
                GPIO.output(self.config.IN2, GPIO.HIGH)
                GPIO.output(self.config.IN3, GPIO.HIGH)
                GPIO.output(self.config.IN4, GPIO.LOW)
                print("[电机] ↺ 左转")
                
            elif direction == MotorState.RIGHT:
                # 左轮正转，右轮反转（原地右转）
                GPIO.output(self.config.IN1, GPIO.HIGH)
                GPIO.output(self.config.IN2, GPIO.LOW)
                GPIO.output(self.config.IN3, GPIO.LOW)
                GPIO.output(self.config.IN4, GPIO.HIGH)
                print("[电机] ↻ 右转")
            
            # 启动PWM
            self.pwm_a.ChangeDutyCycle(self.current_speed)
            self.pwm_b.ChangeDutyCycle(self.current_speed)
            
            # 如果指定了持续时间，启动定时停止
            if duration and duration > 0:
                threading.Timer(duration, self.stop).start()
    
    def _stop_immediate(self):
        """立即停止（内部方法，需要加锁调用）"""
        # 先停PWM（占空比0）
        if self.pwm_a:
            self.pwm_a.ChangeDutyCycle(0)
        if self.pwm_b:
            self.pwm_b.ChangeDutyCycle(0)
        # 再停方向
        GPIO.output(self.config.IN1, GPIO.LOW)
        GPIO.output(self.config.IN2, GPIO.LOW)
        GPIO.output(self.config.IN3, GPIO.LOW)
        GPIO.output(self.config.IN4, GPIO.LOW)
    
    def stop(self):
        """外部调用停止"""
        with self._lock:
            self._stop_immediate()
            self.is_moving = False
            self._current_direction = MotorState.STOP
            print("[电机] ◼ 已停止")
    
    def cleanup(self):
        """清理资源"""
        self.stop()
        if self.pwm_a:
            self.pwm_a.stop()
        if self.pwm_b:
            self.pwm_b.stop()
        print("[电机] 资源已释放")


# ==================== 摄像头控制类（修复窗口关闭）====================

class CameraController:
    """
    摄像头控制器 - 修复版
    修复窗口关闭：使用更可靠的窗口管理策略
    """
    
    def __init__(self, camera_index: int = 0):
        self.camera_index = camera_index
        
        self.cap = None
        self.state = CameraState.CLOSED
        self._lock = threading.Lock()
        
        # 帧捕获线程
        self._frame_thread = None
        self._is_capturing = False
        self.latest_frame = None
        
        # 窗口控制标志
        self.window_should_show = False      # 是否应该显示窗口
        self.window_actually_showing = False  # 窗口当前是否实际在显示
        
        self.preview_window_name = "SmartCar Camera"
        
        print("[摄像头] 控制器初始化完成")
    
    def open(self) -> bool:
        """开启摄像头"""
        with self._lock:
            if self.state in [CameraState.OPEN, CameraState.PREVIEWING]:
                print("[摄像头] 已经是开启状态")
                return True
            
            try:
                # 尝试多种后端
                for backend in [cv2.CAP_V4L2, cv2.CAP_V4L, cv2.CAP_ANY]:
                    self.cap = cv2.VideoCapture(self.camera_index, backend)
                    if self.cap.isOpened():
                        break
                
                if not self.cap or not self.cap.isOpened():
                    raise RuntimeError("无法打开摄像头设备")
                
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                
                # 预热，确保能读到帧
                for _ in range(15):
                    ret, _ = self.cap.read()
                    if not ret:
                        time.sleep(0.05)
                
                # 验证能读到帧
                ret, test_frame = self.cap.read()
                if not ret or test_frame is None:
                    raise RuntimeError("摄像头无法读取帧")
                
                self.state = CameraState.OPEN
                self._is_capturing = True
                self._frame_thread = threading.Thread(target=self._capture_loop, daemon=True)
                self._frame_thread.start()
                
                print(f"[摄像头] ✅ 已开启 (分辨率: {test_frame.shape[1]}x{test_frame.shape[0]})")
                return True
                
            except Exception as e:
                self.state = CameraState.ERROR
                print(f"[摄像头] ❌ 开启失败: {e}")
                return False
    
    def _capture_loop(self):
        """后台持续捕获帧"""
        while self._is_capturing and self.cap and self.cap.isOpened():
            try:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    self.latest_frame = frame
            except:
                pass
            time.sleep(0.033)
    
    def request_show_window(self):
        """请求显示窗口（由语音触发调用）"""
        if self.state == CameraState.CLOSED:
            print("[摄像头] 请先开启摄像头")
            return False
        
        self.window_should_show = True
        print("[摄像头] 📷 请求显示窗口")
        return True
    
    def request_hide_window(self):
        """请求关闭窗口（由语音触发调用）"""
        self.window_should_show = False
        print("[摄像头] 📷 请求关闭窗口")
    
    def process_window(self):
        """
        主线程调用此方法处理窗口显示/关闭
        修复：更可靠的窗口创建/销毁逻辑
        """
        # 情况1：需要显示窗口
        if self.window_should_show:
            if self.latest_frame is not None:
                # 如果窗口不存在，创建它
                if not self.window_actually_showing:
                    cv2.namedWindow(self.preview_window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(self.preview_window_name, 640, 480)
                    cv2.moveWindow(self.preview_window_name, 100, 100)
                    self.window_actually_showing = True
                    self.state = CameraState.PREVIEWING
                    print("[摄像头] 🖥️  窗口已创建")
                
                # 显示帧
                frame = self.latest_frame.copy()
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(frame, timestamp, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                cv2.imshow(self.preview_window_name, frame)
                cv2.waitKey(1)  # 必须调用，刷新窗口
        
        # 情况2：不需要显示窗口，销毁它
        else:
            if self.window_actually_showing:
                # 修复：多次调用destroyWindow和waitKey确保窗口关闭
                cv2.destroyWindow(self.preview_window_name)
                cv2.waitKey(50)
                cv2.destroyWindow(self.preview_window_name)
                cv2.waitKey(50)
                
                # 额外保险：如果还有窗口，全部销毁
                cv2.destroyAllWindows()
                cv2.waitKey(100)
                
                self.window_actually_showing = False
                if self.state == CameraState.PREVIEWING:
                    self.state = CameraState.OPEN
                print("[摄像头] 🖥️  窗口已关闭")
    
    def close(self):
        """完全关闭摄像头"""
        with self._lock:
            # 标记不显示窗口
            self.window_should_show = False
            
            # 销毁窗口
            if self.window_actually_showing:
                cv2.destroyAllWindows()
                cv2.waitKey(200)
                self.window_actually_showing = False
            
            # 停止帧捕获
            self._is_capturing = False
            if self._frame_thread and self._frame_thread.is_alive():
                self._frame_thread.join(timeout=2.0)
            
            # 释放摄像头
            if self.cap:
                try:
                    self.cap.release()
                except:
                    pass
                self.cap = None
            
            # 最终清理
            cv2.destroyAllWindows()
            cv2.waitKey(100)
            
            self.state = CameraState.CLOSED
            self.latest_frame = None
            print("[摄像头] ✅ 已关闭")
    
    def get_status(self) -> str:
        return f"状态: {self.state.value} | 窗口显示: {self.window_actually_showing}"


# ==================== 超声波传感器类 ====================

class UltrasonicSensor:
    """HC-SR04超声波距离传感器"""
    
    SPEED_OF_SOUND_CM_US = 0.01715
    
    def __init__(self, config: PinConfig):
        self.config = config
        self.is_enabled = False
        self._monitor_thread = None
        self._stop_monitor = False
        self._last_distance = 999.0
        self._lock = threading.Lock()
        self._on_obstacle_callback = None
        
        self._setup_gpio()
    
    def _setup_gpio(self):
        GPIO.setup(self.config.PC6_ULTRASONIC_EN, GPIO.OUT)
        GPIO.output(self.config.PC6_ULTRASONIC_EN, GPIO.LOW)
        GPIO.setup(self.config.ULTRASONIC_TRIG, GPIO.OUT)
        GPIO.output(self.config.ULTRASONIC_TRIG, GPIO.LOW)
        GPIO.setup(self.config.ULTRASONIC_ECHO, GPIO.IN)
        print("[超声波] GPIO初始化完成")
    
    def enable(self):
        GPIO.output(self.config.PC6_ULTRASONIC_EN, GPIO.HIGH)
        self.is_enabled = True
        print("[超声波] ✅ 已启用")
    
    def disable(self):
        self._stop_monitor = True
        if self._monitor_thread and self._monitor_thread.is_alive():
            self._monitor_thread.join(timeout=1.0)
        GPIO.output(self.config.PC6_ULTRASONIC_EN, GPIO.LOW)
        self.is_enabled = False
        print("[超声波] ✅ 已禁用")
    
    def measure_distance(self) -> float:
        if not self.is_enabled:
            return 999.0
        
        with self._lock:
            GPIO.output(self.config.ULTRASONIC_TRIG, GPIO.LOW)
            time.sleep(0.000002)
            GPIO.output(self.config.ULTRASONIC_TRIG, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.config.ULTRASONIC_TRIG, GPIO.LOW)
            
            start_time = time.time()
            timeout = start_time + 0.04
            
            while GPIO.input(self.config.ULTRASONIC_ECHO) == GPIO.LOW:
                start_time = time.time()
                if start_time > timeout:
                    return 999.0
            
            end_time = time.time()
            timeout = end_time + 0.04
            
            while GPIO.input(self.config.ULTRASONIC_ECHO) == GPIO.HIGH:
                end_time = time.time()
                if end_time > timeout:
                    return 999.0
            
            duration = (end_time - start_time) * 1000000
            distance = duration * self.SPEED_OF_SOUND_CM_US
            
            if distance < 2 or distance > 400:
                return 999.0
            
            self._last_distance = distance
            return distance
    
    def get_distance(self) -> float:
        return self._last_distance
    
    def start_monitoring(self, interval: float = 0.1, threshold: float = 10.0):
        self._stop_monitor = False
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop,
            args=(interval, threshold),
            daemon=True
        )
        self._monitor_thread.start()
        print(f"[超声波] 🔄 开始后台监控 (阈值{threshold}cm)")
    
    def _monitor_loop(self, interval: float, threshold: float):
        while not self._stop_monitor and self.is_enabled:
            distance = self.measure_distance()
            if distance < threshold and self._on_obstacle_callback:
                print(f"\n[超声波] ⚠️  障碍物! 距离: {distance:.1f}cm")
                self._on_obstacle_callback()
            time.sleep(interval)
    
    def register_obstacle_callback(self, callback: Callable):
        self._on_obstacle_callback = callback
        print("[超声波] 已注册避障回调")
    
    def cleanup(self):
        self.disable()
        print("[超声波] 资源已释放")


# ==================== 语音触发检测器 ====================

class ParallelVoiceDetector:
    """并行语音触发检测器"""
    
    def __init__(self, config: PinConfig):
        self.config = config
        self.is_listening = False
        self._callbacks = {}
        self._last_trigger_time = {}
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
        print(f"[语音] 已配置引脚")
    
    def register_callback(self, pin: int, callback: Callable):
        self._callbacks[pin] = callback
        name = self._pin_names.get(pin, f"引脚{pin}")
        print(f"[语音] 已注册 [{name}] GPIO{pin}")
    
    def _create_rising_callback(self, pin: int):
        def callback(channel):
            current_time = int(time.time() * 1000)
            if current_time - self._last_trigger_time[pin] < self._debounce_ms:
                return
            self._last_trigger_time[pin] = current_time
            
            if GPIO.input(pin) == GPIO.HIGH:
                name = self._pin_names.get(pin, f"引脚{pin}")
                print(f"\n[语音] 🔊 [{name}] GPIO{pin}")
                if pin in self._callbacks:
                    try:
                        self._callbacks[pin]()
                    except Exception as e:
                        print(f"[语音] ❌ 错误: {e}")
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
                print(f"\n[语音] 📷 [{name}] 上升沿 → 开窗口")
                action = "open"
            else:
                print(f"\n[语音] 📷 [{name}] 下降沿 → 关窗口")
                action = "close"
            
            if pin in self._callbacks:
                try:
                    self._callbacks[pin](action)
                except Exception as e:
                    print(f"[语音] ❌ 错误: {e}")
        return callback
    
    def start_listening(self):
        self.is_listening = True
        for pin in self.config.VOICE_PINS:
            GPIO.add_event_detect(pin, GPIO.RISING,
                callback=self._create_rising_callback(pin), bouncetime=50)
        GPIO.add_event_detect(self.config.PA2_CAMERA, GPIO.BOTH,
            callback=self._create_both_callback(self.config.PA2_CAMERA), bouncetime=50)
        print("[语音] 🎙️  开始监听")
    
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


# ==================== 主控制类 ====================

class SmartCarController:
    """智能小车主控制器 - 纯语音控制"""
    
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
        self._is_avoiding = False
        
        self.ultrasonic.register_obstacle_callback(self._on_obstacle_detected)
        self._register_voice_callbacks()
        
        print("\n" + "="*50)
        print("    🚗 智能小车控制系统初始化完成")
        print("="*50)
    
    def _register_voice_callbacks(self):
        self.voice.register_callback(self.config.VOICE_FORWARD, self._cmd_forward)
        self.voice.register_callback(self.config.VOICE_STOP, self._cmd_stop)
        self.voice.register_callback(self.config.VOICE_BACKWARD, self._cmd_backward)
        self.voice.register_callback(self.config.VOICE_LEFT, self._cmd_left)
        self.voice.register_callback(self.config.VOICE_RIGHT, self._cmd_right)
        self.voice.register_callback(self.config.PA2_CAMERA, self._cmd_camera_control)
    
    def _on_obstacle_detected(self):
        if self._is_avoiding:
            return
        if self.motor.current_direction != MotorState.FORWARD:
            return
        
        self._is_avoiding = True
        print("\n" + "!"*40)
        print("[避障] 🚧 执行避障!")
        print("!"*40)
        
        self.motor.stop()
        time.sleep(0.3)
        
        self.motor.move(MotorState.BACKWARD)
        time.sleep(0.5)
        self.motor.stop()
        time.sleep(0.2)
        
        turn = random.choice([MotorState.LEFT, MotorState.RIGHT])
        print(f"[避障] 🔄 随机{'左转' if turn == MotorState.LEFT else '右转'}")
        self.motor.move(turn)
        time.sleep(0.8)
        self.motor.stop()
        time.sleep(0.2)
        
        print("[避障] ✅ 恢复前进")
        self.motor.move(MotorState.FORWARD)
        self._is_avoiding = False
    
    def _cmd_forward(self):
        self.ultrasonic.enable()
        self.ultrasonic.start_monitoring(interval=0.1, threshold=10.0)
        self.motor.move(MotorState.FORWARD)
    
    def _cmd_stop(self):
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
                if success:
                    self.camera.request_show_window()
                else:
                    print("[错误] 摄像头开启失败")
            else:
                self.camera.request_show_window()
                
        elif action == "close":
            self.camera.request_hide_window()
    
    def start(self):
        """启动语音监听主循环"""
        self.voice.start_listening()
        
        print("\n" + "-"*50)
        print("系统运行中，语音指令如下：")
        print("  [前进]     → GPIO17 上升沿（PA1）+ 启用超声波")
        print("  [停止]     → GPIO22 上升沿（PA4）+ 关闭超声波")
        print("  [后退]     → GPIO23 上升沿（PC4）")
        print("  [左转]     → GPIO24 上升沿（PB5）")
        print("  [右转]     → GPIO25 上升沿（PB6）")
        print("  [开窗口]   → GPIO27 上升沿（PA2 低→高）")
        print("  [关窗口]   → GPIO27 下降沿（PA2 高→低）")
        print("-"*50)
        print("按 Ctrl+C 退出程序\n")
        
        try:
            while self.is_running:
                # 主线程处理摄像头窗口显示/关闭
                self.camera.process_window()
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n[系统] 检测到键盘中断")
        finally:
            self.shutdown()
    
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


# ==================== 程序入口 ====================

if __name__ == "__main__":
    def signal_handler(sig, frame):
        print("\n[信号] 收到中断信号")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    car = SmartCarController()
    car.start()
