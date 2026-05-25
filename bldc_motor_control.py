#!/usr/bin/env python3
"""
Система управления BLDC-двигателем KMTech MG4010E v3
Протокол: CAN
Алгоритм: Field Oriented Control (FOC)
Функции: Получение данных о токах, визуализация в реальном времени
"""

import can
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import time
import threading
import struct
import argparse


class BLDCMotorController:
    """Класс для управления BLDC-двигателем через CAN интерфейс"""
    
    # CAN ID для двигателя KMTech MG4010E v3
    # (могут быть изменены в зависимости от конфигурации двигателя)
    CAN_ID_CURRENT_FEEDBACK = 0x201  # ID для получения данных о токах
    CAN_ID_COMMAND = 0x100           # ID для отправки команд
    
    def __init__(self, channel='can0', bitrate=500000):
        """
        Инициализация контроллера
        
        Args:
            channel: CAN интерфейс (can0, vcan0, usbcan и т.д.)
            bitrate: Скорость передачи в бод (по умолчанию 500 кбит/с)
        """
        self.channel = channel
        self.bitrate = bitrate
        self.bus = None
        self.current_iq = [0.0, 0.0]  # Токи Id, Iq
        self.current_abc = [0.0, 0.0, 0.0]  # Токи Ia, Ib, Ic
        self.running = False
        self.data_lock = threading.Lock()
        
    def connect(self):
        """Подключение к CAN шине"""
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype='socketcan',
                bitrate=self.bitrate
            )
            print(f"✓ Подключено к CAN шине: {self.channel} ({self.bitrate} бит/с)")
            return True
        except Exception as e:
            print(f"✗ Ошибка подключения к CAN шине: {e}")
            print("  Запуск в режиме симуляции...")
            self.bus = None
            return False
    
    def disconnect(self):
        """Отключение от CAN шины"""
        if self.bus:
            self.bus.shutdown()
            print("✓ Отключено от CAN шины")
    
    def receive_message(self, timeout=1.0):
        """
        Получение сообщения от двигателя
        
        Args:
            timeout: Таймаут ожидания сообщения в секундах
            
        Returns:
            Сообщение CAN или None
        """
        if self.bus is None:
            return None
        
        try:
            msg = self.bus.recv(timeout=timeout)
            return msg
        except Exception as e:
            print(f"Ошибка получения сообщения: {e}")
            return None
    
    def parse_current_data(self, msg):
        """
        Парсинг данных о токах из CAN сообщения
        
        Формат данных зависит от спецификации двигателя KMTech MG4010E v3
        Предполагаемый формат:
        - Байты 0-1: Ток Id (signed int16, масштаб 0.01A)
        - Байты 2-3: Ток Iq (signed int16, масштаб 0.01A)
        - Байты 4-5: Ток Ia (опционально)
        - Байты 6-7: Ток Ib (опционально)
        
        Args:
            msg: CAN сообщение
            
        Returns:
            dict с данными о токах
        """
        if msg is None or len(msg.data) < 4:
            return None
        
        data = msg.data
        
        # Парсинг токов Id и Iq (DQ frame)
        id_raw = struct.unpack('<h', bytes([data[0], data[1]]))[0]
        iq_raw = struct.unpack('<h', bytes([data[2], data[3]]))[0]
        
        id_current = id_raw * 0.01  # Преобразование в Амперы
        iq_current = iq_raw * 0.01
        
        # Парсинг токов фаз (ABC frame), если доступны
        ia_current = 0.0
        ib_current = 0.0
        ic_current = 0.0
        
        if len(data) >= 6:
            ia_raw = struct.unpack('<h', bytes([data[4], data[5]]))[0]
            ia_current = ia_raw * 0.01
        
        if len(data) >= 8:
            ib_raw = struct.unpack('<h', bytes([data[6], data[7]]))[0]
            ib_current = ib_raw * 0.01
            ic_current = -(ia_current + ib_current)  # Третий ток по закону Кирхгофа
        
        with self.data_lock:
            self.current_iq = [id_current, iq_current]
            self.current_abc = [ia_current, ib_current, ic_current]
        
        return {
            'id': id_current,
            'iq': iq_current,
            'ia': ia_current,
            'ib': ib_current,
            'ic': ic_current,
            'timestamp': time.time()
        }
    
    def send_command(self, command_id, data):
        """
        Отправка команды двигателю
        
        Args:
            command_id: CAN ID команды
            data: Данные команды (bytes)
        """
        if self.bus is None:
            return False
        
        try:
            msg = can.Message(
                arbitration_id=command_id,
                data=data,
                is_extended_id=False
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"Ошибка отправки команды: {e}")
            return False
    
    def simulate_data(self):
        """Генерация симулированных данных для тестирования без оборудования"""
        import math
        
        t = time.time()
        
        # Симуляция синусоидальных токов с шумом
        base_freq = 2.0  # Гц
        noise_amplitude = 0.1
        
        id_sim = 1.5 * math.sin(2 * math.pi * base_freq * t) + noise_amplitude * (0.5 - hash(str(t)) % 100 / 100)
        iq_sim = 2.0 * math.cos(2 * math.pi * base_freq * t) + noise_amplitude * (0.5 - hash(str(t * 1.1)) % 100 / 100)
        
        # Преобразование Park inverse для получения фазных токов
        theta = 2 * math.pi * base_freq * t
        ia_sim = id_sim * math.cos(theta) - iq_sim * math.sin(theta)
        ib_sim = id_sim * math.cos(theta - 2*math.pi/3) - iq_sim * math.sin(theta - 2*math.pi/3)
        ic_sim = -(ia_sim + ib_sim)
        
        with self.data_lock:
            self.current_iq = [id_sim, iq_sim]
            self.current_abc = [ia_sim, ib_sim, ic_sim]
        
        return {
            'id': id_sim,
            'iq': iq_sim,
            'ia': ia_sim,
            'ib': ib_sim,
            'ic': ic_sim,
            'timestamp': t
        }


class CurrentVisualizer:
    """Класс для визуализации токов двигателя в реальном времени"""
    
    def __init__(self, controller, max_points=200):
        """
        Инициализация визуализатора
        
        Args:
            controller: Экземпляр BLDCMotorController
            max_points: Максимальное количество точек на графике
        """
        self.controller = controller
        self.max_points = max_points
        
        # Хранилища данных
        self.time_data = deque(maxlen=max_points)
        self.id_data = deque(maxlen=max_points)
        self.iq_data = deque(maxlen=max_points)
        self.ia_data = deque(maxlen=max_points)
        self.ib_data = deque(maxlen=max_points)
        self.ic_data = deque(maxlen=max_points)
        
        # Настройка matplotlib
        plt.style.use('seaborn-v0_8-darkgrid')
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.suptitle('BLDC Motor KMTech MG4010E v3 - FOC Current Monitoring', 
                         fontsize=16, fontweight='bold')
        
        # Создание подграфиков
        self.ax_dq = self.fig.add_subplot(2, 1, 1)
        self.ax_abc = self.fig.add_subplot(2, 1, 2)
        
        # Инициализация линий
        self.line_id, = self.ax_dq.plot([], [], 'b-', linewidth=2, label='Id (A)')
        self.line_iq, = self.ax_dq.plot([], [], 'r-', linewidth=2, label='Iq (A)')
        
        self.line_ia, = self.ax_abc.plot([], [], 'g-', linewidth=2, label='Ia (A)')
        self.line_ib, = self.ax_abc.plot([], [], 'm-', linewidth=2, label='Ib (A)')
        self.line_ic, = self.ax_abc.plot([], [], 'c-', linewidth=2, label='Ic (A)')
        
        # Настройка осей
        self.setup_axes()
        
        # Добавление легенд
        self.ax_dq.legend(loc='upper right')
        self.ax_abc.legend(loc='upper right')
        
        # Текстовые метрики
        self.text_metrics = self.fig.text(0.02, 0.98, '', 
                                          transform=self.fig.transFigure,
                                          fontsize=10, verticalalignment='top',
                                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        self.start_time = None
        
    def setup_axes(self):
        """Настройка осей графиков"""
        # График DQ токов
        self.ax_dq.set_ylabel('Current (A)', fontsize=12)
        self.ax_dq.set_title('D-Q Frame Currents (Rotating Reference)', fontsize=14)
        self.ax_dq.set_ylim(-5, 5)
        self.ax_dq.grid(True, alpha=0.3)
        
        # График ABC токов
        self.ax_abc.set_xlabel('Time (s)', fontsize=12)
        self.ax_abc.set_ylabel('Current (A)', fontsize=12)
        self.ax_abc.set_title('Three-Phase Currents (Stationary Reference)', fontsize=14)
        self.ax_abc.set_ylim(-5, 5)
        self.ax_abc.grid(True, alpha=0.3)
        
    def init_animation(self):
        """Инициализация анимации"""
        self.time_data.clear()
        self.id_data.clear()
        self.iq_data.clear()
        self.ia_data.clear()
        self.ib_data.clear()
        self.ic_data.clear()
        
        self.line_id.set_data([], [])
        self.line_iq.set_data([], [])
        self.line_ia.set_data([], [])
        self.line_ib.set_data([], [])
        self.line_ic.set_data([], [])
        
        self.start_time = time.time()
        
        return [self.line_id, self.line_iq, self.line_ia, self.line_ib, self.line_ic, self.text_metrics]
    
    def update(self, frame):
        """
        Обновление данных графиков
        
        Args:
            frame: Номер кадра (не используется)
            
        Returns:
            Список обновленных объектов
        """
        # Получение данных от контроллера
        if self.controller.bus is not None:
            msg = self.controller.receive_message(timeout=0.1)
            if msg and msg.arbitration_id == self.controller.CAN_ID_CURRENT_FEEDBACK:
                data = self.controller.parse_current_data(msg)
            else:
                data = self.controller.simulate_data()
        else:
            data = self.controller.simulate_data()
        
        if data is None:
            return [self.line_id, self.line_iq, self.line_ia, self.line_ib, self.line_ic, self.text_metrics]
        
        # Добавление данных
        current_time = data['timestamp'] - self.start_time if self.start_time else 0
        self.time_data.append(current_time)
        self.id_data.append(data['id'])
        self.iq_data.append(data['iq'])
        self.ia_data.append(data['ia'])
        self.ib_data.append(data['ib'])
        self.ic_data.append(data['ic'])
        
        # Обновление линий
        time_list = list(self.time_data)
        
        self.line_id.set_data(time_list, list(self.id_data))
        self.line_iq.set_data(time_list, list(self.iq_data))
        self.line_ia.set_data(time_list, list(self.ia_data))
        self.line_ib.set_data(time_list, list(self.ib_data))
        self.line_ic.set_data(time_list, list(self.ic_data))
        
        # Автонастройка оси X
        if len(time_list) > 0:
            self.ax_dq.set_xlim(0, time_list[-1] + 0.1)
            self.ax_abc.set_xlim(0, time_list[-1] + 0.1)
        
        # Обновление метрик
        metrics_text = (
            f"Time: {current_time:.2f} s\n"
            f"Id: {data['id']:.3f} A | Iq: {data['iq']:.3f} A\n"
            f"Ia: {data['ia']:.3f} A | Ib: {data['ib']:.3f} A | Ic: {data['ic']:.3f} A\n"
            f"|I| = √(Id² + Iq²) = {((data['id']**2 + data['iq']**2)**0.5):.3f} A"
        )
        self.text_metrics.set_text(metrics_text)
        
        return [self.line_id, self.line_iq, self.line_ia, self.line_ib, self.line_ic, self.text_metrics]
    
    def start(self, interval=50):
        """
        Запуск визуализации
        
        Args:
            interval: Интервал обновления в миллисекундах
        """
        print("\n" + "="*60)
        print("Запуск визуализации токов BLDC-двигателя")
        print("="*60)
        print("Закройте окно графика для остановки программы")
        print("="*60 + "\n")
        
        self.anim = FuncAnimation(
            self.fig, 
            self.update,
            init_func=self.init_animation,
            blit=True,
            interval=interval,
            cache_frame_data=False
        )
        
        plt.tight_layout()
        plt.show()


def main():
    """Основная функция программы"""
    
    # Парсинг аргументов командной строки
    parser = argparse.ArgumentParser(
        description='Система управления BLDC-двигателем KMTech MG4010E v3'
    )
    parser.add_argument(
        '--channel', '-c',
        default='vcan0',
        help='CAN интерфейс (по умолчанию: vcan0)'
    )
    parser.add_argument(
        '--bitrate', '-b',
        type=int,
        default=500000,
        help='Скорость передачи в бод (по умолчанию: 500000)'
    )
    parser.add_argument(
        '--simulate', '-s',
        action='store_true',
        help='Режим симуляции без оборудования'
    )
    
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("BLDC Motor Control System - FOC Algorithm")
    print("Двигатель: KMTech MG4010E v3")
    print("Протокол: CAN")
    print("="*60 + "\n")
    
    # Создание контроллера
    controller = BLDCMotorController(
        channel=args.channel,
        bitrate=args.bitrate
    )
    
    # Подключение или режим симуляции
    if not args.simulate:
        connected = controller.connect()
        if not connected:
            print("\nПереход в режим симуляции...")
    else:
        print("Режим симуляции активирован")
    
    # Создание визуализатора
    visualizer = CurrentVisualizer(controller, max_points=200)
    
    try:
        # Запуск визуализации
        visualizer.start(interval=50)
    except KeyboardInterrupt:
        print("\n\nОстановка программы пользователем...")
    finally:
        # Отключение
        controller.disconnect()
        print("\nПрограмма завершена")


if __name__ == '__main__':
    main()
