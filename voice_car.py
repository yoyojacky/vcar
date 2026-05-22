#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
树莓派4B智能小车控制程序 - 并行语音触发版本
语音模块通过独立GPIO引脚输出高电平信号，树莓派监控各引脚电平判断指令
"""

import RPi.GPIO as GPIO
import cv2
import time
import threading
import sys
import signal
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
    
    # === 语音模块触发引脚（并行输出，每个引脚对应一个指令）===
    # 根据你的图片，这些引脚在语音触发时会输出高电平
    VOICE_WAKEUP: int = 18      # 唤醒词 (对应图中PA_0)
    VOICE_FORWARD: int = 17     # 前进 (对应图中PA_1)
    VOICE_STOP: int = 22       # 停止 (对应图中PA_4)
    VOICE_BACKWARD: int = 23    # 后退 (对应图中PC_4)
    VOICE_LEFT: int = 24       # 左转 (对应图中PB_5)
    VOICE_RIGHT: int = 25      # 右转 (对应图中PB_6)
    VOICE_CAM_ON: int = 27     # 打开摄像头 (对应图中PA_2)
    VOICE_CAM_OFF: int = 26    # 关闭摄像头 (图中未显示，需要确认实际引脚)
    
    # 所有语音触发引脚列表（方便批量操作）
    @property
    def VOICE_PINS(self) -> list:
        return [
            self.VOICE_WAKEUP, self.VOICE_FORWARD, self.VOICE_STOP,
            self.VOICE_BACKWARD, self.VOICE_LEFT, self.VOICE_RIGHT,
            self.VOICE_CAM_ON, self.VOICE_CAM_OFF
        ]


class MotorState(Enum):
    """电机状态"""
    STOP = "停止"
    FORWARD = "前进"
    BACKWARD = "后退"
    LEFT = "左转"
    RIGHT = "右转"


class CameraState(Enum):
    """摄像头状态"""
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
        self.current_speed = 80      # 默认速度 0-100
        self.is_moving = False
        self._lock = threading.Lock()
        self._setup_gpio()
    
    def _setup_gpio(self):
        """初始化GPIO引脚"""
        motor_pins = [
            self.config.IN1, self.config.IN2,
            self.config.IN3, self.config.IN4,
            self.config.ENA, self.config.ENB
        ]
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # 创建PWM，频率1kHz
        self.pwm_a = GPIO.PWM(self.config.ENA, 1000)
        self.pwm_b = GPIO.PWM(self.config.ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)
        print("[电机] GPIO初始化完成")
    
    def set_speed(self, speed: int):
        """设置速度 0-100"""
        speed = max(0, min(100, speed))
        self.current_speed = speed
        self.pwm_a.ChangeDutyCycle(speed)
        self.pwm_b.ChangeDutyCycle(speed)
        print(f"[电机] 速度: {speed}%")
    
    def move(self, direction: MotorState, duration: Optional[float] = None):
        """控制运动方向"""
        with self._lock:
            self._stop_immediate()
            
            if direction == MotorState.STOP:
                self.is_moving = False
                print("[电机] 停止")
                return
            
            self.is_moving = True
            
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
                # 左轮反转，右轮正转 → 原地左转
                GPIO.output(self.config.IN1, GPIO.LOW)
                GPIO.output(self.config.IN2, GPIO.HIGH)
                GPIO.output(self.config.IN3, GPIO.HIGH)
                GPIO.output(self.config.IN4, GPIO.LOW)
                print("[电机] ↺ 左转")
                
            elif direction == MotorState.RIGHT:
                # 左轮正转，右轮反转 → 原地右转
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
        """立即停止（内部方法，需加锁调用）"""
        GPIO.output(self.config.IN1, GPIO.LOW)
        GPIO.output(self.config.IN2, GPIO.LOW)
        GPIO.output(self.config.IN3, GPIO.LOW)
        GPIO.output(self.config.IN4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
    
    def stop(self):
        """外部停止"""
        with self._lock:
            self._stop_immediate()
            self.is_moving = False
            print("[电机] ◼ 已停止")
    
    def cleanup(self):
        """清理资源"""
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
        """开启摄像头"""
        with self._lock:
            if self.state == CameraState.OPEN:
                print("[摄像头] 已经是开启状态")
                return True
            
            try:
                self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.cap.set(cv2.CAP_PROP_FPS, 30)
                
                if not self.cap.isOpened():
                    raise RuntimeError("无法打开摄像头设备")
                
                # 预热：丢弃前5帧
                for _ in range(5):
                    self.cap.read()
                
                self.state = CameraState.OPEN
                self._is_capturing = True
                # 启动后台帧捕获线程
                self._frame_thread = threading.Thread(target=self._capture_loop, daemon=True)
                self._frame_thread.start()
                
                print(f"[摄像头] ✅ 已开启 (索引: {self.camera_index})")
                return True
                
            except Exception as e:
                self.state = CameraState.ERROR
                print(f"[摄像头] ❌ 开启失败: {e}")
                return False
    
    def _capture_loop(self):
        """后台持续捕获帧"""
        while self._is_capturing and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.latest_frame = frame
            time.sleep(0.033)  # ~30fps
    
    def close(self):
        """关闭摄像头"""
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
        """拍照"""
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


# ==================== 并行语音触发检测器 ====================

class ParallelVoiceDetector:
    """
    并行语音触发检测器
    同时监控8个GPIO引脚，哪个变高就执行对应指令
    使用边沿中断检测，CPU占用极低
    """
    
    def __init__(self, config: PinConfig):
        self.config = config
        self.is_listening = False
        self._callbacks: Dict[int, Callable] = {}
        self._last_trigger_time: Dict[int, int] = {}  # 每个引脚独立的防抖时间
        self._debounce_ms = 300  # 防抖300ms，避免信号抖动
        
        # 引脚到指令名称的映射（用于日志显示）
        self._pin_names = {
            config.VOICE_WAKEUP: "唤醒词",
            config.VOICE_FORWARD: "前进",
            config.VOICE_STOP: "停止",
            config.VOICE_BACKWARD: "后退",
            config.VOICE_LEFT: "左转",
            config.VOICE_RIGHT: "右转",
            config.VOICE_CAM_ON: "摄像头开",
            config.VOICE_CAM_OFF: "摄像头关",
        }
        
        self._setup_gpio()
    
    def _setup_gpio(self):
        """配置所有语音触发引脚为输入，下拉电阻"""
        for pin in self.config.VOICE_PINS:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            self._last_trigger_time[pin] = 0
        print(f"[语音] 已配置 {len(self.config.VOICE_PINS)} 个触发引脚")
    
    def register_callback(self, pin: int, callback: Callable):
        """为特定引脚注册回调函数"""
        self._callbacks[pin] = callback
        name = self._pin_names.get(pin, f"引脚{pin}")
        print(f"[语音] 已注册 [{name}] GPIO{pin} 的回调")
    
    def _create_edge_callback(self, pin: int):
        """
        创建边沿检测回调函数（闭包，绑定特定引脚）
        """
        def callback(channel):
            current_time = int(time.time() * 1000)
            
            # 防抖检查
            if current_time - self._last_trigger_time[pin] < self._debounce_ms:
                return
            
            self._last_trigger_time[pin] = current_time
            
            # 确认确实是高电平（排除干扰）
            if GPIO.input(pin) == GPIO.HIGH:
                name = self._pin_names.get(pin, f"引脚{pin}")
                print(f"\n{'='*40}")
                print(f"[语音] 🔊 检测到触发: [{name}] GPIO{pin}")
                print(f"{'='*40}")
                
                # 执行回调
                if pin in self._callbacks:
                    try:
                        self._callbacks[pin]()
                    except Exception as e:
                        print(f"[语音] ❌ 回调执行错误: {e}")
        
        return callback
    
    def start_listening(self):
        """开始监听所有引脚的上升沿"""
        self.is_listening = True
        
        for pin in self.config.VOICE_PINS:
            # 为每个引脚添加边沿检测中断
            # bouncetime是硬件防抖（毫秒），和软件防抖双重保险
            GPIO.add_event_detect(
                pin,
                GPIO.RISING,           # 检测上升沿（低→高）
                callback=self._create_edge_callback(pin),
                bouncetime=50          # 硬件防抖50ms
            )
        
        print("[语音] 🎙️  开始监听所有触发引脚...")
        print("[语音] 等待语音指令中...")
    
    def stop_listening(self):
        """停止监听"""
        self.is_listening = False
        for pin in self.config.VOICE_PINS:
            try:
                GPIO.remove_event_detect(pin)
            except:
                pass
        print("[语音] 🛑 已停止监听")
    
    def manual_trigger(self, pin: int):
        """手动触发某个引脚（用于测试）"""
        if pin in self._callbacks:
            self._callbacks[pin]()


# ==================== 主控制类 ====================

class SmartCarController:
    """
    智能小车主控制器
    """
    
    def __init__(self):
        # 初始化GPIO（BCM编码）
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self.config = PinConfig()
        self.motor = MotorController(self.config)
        self.camera = CameraController(camera_index=0)
        self.voice = ParallelVoiceDetector(self.config)
        
        self.is_camera_on = False
        self.is_running = True
        
        # 注册所有语音引脚对应的回调
        self._register_voice_callbacks()
        
        print("\n" + "="*50)
        print("    🚗 智能小车控制系统初始化完成")
        print("="*50)
    
    def _register_voice_callbacks(self):
        """
        将每个语音触发引脚绑定到对应的功能
        """
        # 唤醒词 - GPIO18
        self.voice.register_callback(
            self.config.VOICE_WAKEUP,
            self._cmd_wakeup
        )
        
        # 前进 - GPIO17
        self.voice.register_callback(
            self.config.VOICE_FORWARD,
            self._cmd_forward
        )
        
        # 停止 - GPIO22
        self.voice.register_callback(
            self.config.VOICE_STOP,
            self._cmd_stop
        )
        
        # 后退 - GPIO23
        self.voice.register_callback(
            self.config.VOICE_BACKWARD,
            self._cmd_backward
        )
        
        # 左转 - GPIO24
        self.voice.register_callback(
            self.config.VOICE_LEFT,
            self._cmd_left
        )
        
        # 右转 - GPIO25
        self.voice.register_callback(
            self.config.VOICE_RIGHT,
            self._cmd_right
        )
        
        # 打开摄像头 - GPIO27
        self.voice.register_callback(
            self.config.VOICE_CAM_ON,
            self._cmd_camera_on
        )
        
        # 关闭摄像头 - GPIO26
        self.voice.register_callback(
            self.config.VOICE_CAM_OFF,
            self._cmd_camera_off
        )
    
    # ===== 指令处理函数 =====
    
    def _cmd_wakeup(self):
        """唤醒词 - 系统就绪提示"""
        print("[指令] 👋 唤醒词识别，系统就绪")
        # 可以在这里播放提示音
        # import os
        # os.system("aplay /home/pi/sounds/wakeup.wav &")
    
    def _cmd_forward(self):
        """前进"""
        self.motor.move(MotorState.FORWARD)
    
    def _cmd_stop(self):
        """停止"""
        self.motor.stop()
    
    def _cmd_backward(self):
        """后退"""
        self.motor.move(MotorState.BACKWARD)
    
    def _cmd_left(self):
        """左转"""
        self.motor.move(MotorState.LEFT)
    
    def _cmd_right(self):
        """右转"""
        self.motor.move(MotorState.RIGHT)
    
    def _cmd_camera_on(self):
        """开启摄像头"""
        if not self.is_camera_on:
            success = self.camera.open()
            self.is_camera_on = success
            if not success:
                print("[错误] 摄像头开启失败")
        else:
            print("[提示] 摄像头已经是开启状态")
    
    def _cmd_camera_off(self):
        """关闭摄像头"""
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
        print("  [唤醒词] → 系统就绪")
        print("  [前进]   → 小车前进")
        print("  [停止]   → 小车停止")
        print("  [后退]   → 小车后退")
        print("  [左转]   → 小车左转")
        print("  [右转]   → 小车右转")
        print("  [摄像头开] → 开启USB摄像头")
        print("  [摄像头关] → 关闭USB摄像头")
        print("-"*50)
        print("按 Ctrl+C 退出程序\n")
        
        try:
            while self.is_running:
                time.sleep(1)
                # 可以在这里添加状态心跳或 watchdog
        except KeyboardInterrupt:
            print("\n[系统] 检测到键盘中断")
        finally:
            self.shutdown()
    
    def manual_test(self, pin_name: str):
        """手动测试某个功能"""
        pin_map = {
            'wakeup': self.config.VOICE_WAKEUP,
            'forward': self.config.VOICE_FORWARD,
            'stop': self.config.VOICE_STOP,
            'backward': self.config.VOICE_BACKWARD,
            'left': self.config.VOICE_LEFT,
            'right': self.config.VOICE_RIGHT,
            'cam_on': self.config.VOICE_CAM_ON,
            'cam_off': self.config.VOICE_CAM_OFF,
        }
        
        pin = pin_map.get(pin_name.lower())
        if pin:
            print(f"\n[测试] 手动触发: {pin_name}")
            self.voice.manual_trigger(pin)
        else:
            print(f"[测试] 未知命令: {pin_name}")
            print(f"可用命令: {list(pin_map.keys())}")
    
    def get_status(self) -> str:
        return f"""
========== 系统状态 ==========
电机: {'运行中' if self.motor.is_moving else '停止'} | 速度: {self.motor.current_speed}%
摄像头: {self.camera.get_status()}
语音监听: {'运行中' if self.voice.is_listening else '停止'}
==============================
"""
    
    def shutdown(self):
        """安全关闭"""
        print("\n" + "="*50)
        print("[系统] 正在安全关闭...")
        
        self.is_running = False
        self.motor.cleanup()
        
        if self.is_camera_on:
            self.camera.close()
        
        self.voice.stop_listening()
        GPIO.cleanup()
        
        print("[系统] ✅ 已安全关闭，GPIO已释放")
        print("="*50 + "\n")
        sys.exit(0)


# ==================== 测试与运行 ====================

def test_all_functions(car: SmartCarController):
    """顺序测试所有功能"""
    print("\n========== 开始硬件测试 ==========\n")
    
    # 测试电机
    print("[测试] 电机测试...")
    car.motor.set_speed(60)
    
    for direction in [MotorState.FORWARD, MotorState.BACKWARD, 
                      MotorState.LEFT, MotorState.RIGHT]:
        print(f"\n--> 测试: {direction.value}")
        car.motor.move(direction)
        time.sleep(1.5)
        car.motor.stop()
        time.sleep(0.5)
    
    # 测试摄像头
    print("\n[测试] 摄像头测试...")
    car._cmd_camera_on()
    time.sleep(2)
    car.camera.take_photo()
    time.sleep(1)
    car._cmd_camera_off()
    
    print("\n========== 测试完成 ==========\n")


def interactive_test(car: SmartCarController):
    """交互式测试模式"""
    print("""
========== 交互测试模式 ==========
输入命令测试对应功能:
  w / wakeup    - 唤醒词
  f / forward   - 前进
  b / backward  - 后退
  l / left      - 左转
  r / right     - 右转
  s / stop      - 停止
  o / cam_on    - 开摄像头
  p / cam_off   - 关摄像头
  t / photo     - 拍照
  q / quit      - 退出
==================================
""")
    
    while True:
        try:
            cmd = input("命令> ").strip().lower()
            if cmd in ['q', 'quit', 'exit']:
                break
            elif cmd in ['w', 'wakeup']:
                car.manual_test('wakeup')
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
                car.manual_test('cam_on')
            elif cmd in ['p', 'cam_off', 'off']:
                car.manual_test('cam_off')
            elif cmd in ['t', 'photo', 'pic']:
                car.camera.take_photo()
            else:
                print("未知命令")
        except KeyboardInterrupt:
            break
    
    car.shutdown()


def main():
    """主程序入口"""
    car = SmartCarController()
    
    # 参数解析
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "1"
    
    if mode == "1" or mode == "voice":
        # 语音控制模式（默认）
        car.start()
        
    elif mode == "2" or mode == "test":
        # 自动测试模式
        test_all_functions(car)
        car.shutdown()
        
    elif mode == "3" or mode == "interactive":
        # 交互测试模式
        interactive_test(car)
        
    else:
        print(f"未知模式: {mode}")
        print("用法: python3 smartcar.py [1|voice|2|test|3|interactive]")
        car.shutdown()


if __name__ == "__main__":
    # 信号处理确保Ctrl+C安全退出
    def signal_handler(sig, frame):
        print("\n[信号] 收到中断信号")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    main()
