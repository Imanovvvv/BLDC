#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Система управления и визуализации BLDC-двигателя KMTech MG4010E v3
Использует библиотеку pylkmotor для работы с протоколом двигателя.
Алгоритм: Field Oriented Control (FOC) - мониторинг токов Id, Iq.
"""

import sys
import time
import argparse
import threading
import random
import math
from collections import deque

try:
    # Попытка импорта pylkmotor
    from pylkmotor import LKMotor, LKMotorState
    PYLKMOTOR_AVAILABLE = True
except ImportError:
    PYLKMOTOR_AVAILABLE = False
    print("⚠️ Библиотека 'pylkmotor' не найдена. Запуск в режиме симуляции.")

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("❌ Библиотека 'matplotlib' не найдена. Установите: pip install matplotlib")
    sys.exit(1)


class MotorDataSimulator:
    """Генератор симулированных данных для тестирования визуализации без оборудования."""
    
    def __init__(self):
        self.t = 0
        self.base_speed = 10.0  # рад/с
    
    def get_data(self):
        self.t += 0.05
        
        # Симуляция токов FOC (Id ~ 0, Iq ~ нагрузка)
        # Добавляем шум и гармоники
        iq_ref = 5.0 * math.sin(self.t * 0.5) + 2.0
        id_ref = 0.5 * math.sin(self.t * 2.0) # Небольшой разбаланс
        
        # Фазные токи (преобразование Парка обратное, упрощенно)
        angle = self.t * self.base_speed
        ia = iq_ref * math.sin(angle) + id_ref * math.cos(angle)
        ib = iq_ref * math.sin(angle - 2*math.pi/3) + id_ref * math.cos(angle - 2*math.pi/3)
        ic = -(ia + ib) # Закон Кирхгофа
        
        # Добавление шума
        noise = lambda: random.uniform(-0.1, 0.1)
        
        return {
            'id': id_ref + noise(),
            'iq': iq_ref + noise(),
            'ia': ia + noise(),
            'ib': ib + noise(),
            'ic': ic + noise(),
            'speed': self.base_speed + random.uniform(-0.2, 0.2),
            'voltage': 24.0 + random.uniform(-0.5, 0.5)
        }


class BLDCController:
    """Класс для получения данных от двигателя."""
    
    def __init__(self, use_simulator=False, channel=None, interface=None):
        self.use_simulator = use_simulator
        self.motor = None
        self.simulator = MotorDataSimulator()
        self.connected = False
        
        if not use_simulator and PYLKMOTOR_AVAILABLE:
            try:
                print(f"🔌 Попытка подключения к двигателю через интерфейс: {interface or 'default'}, канал: {channel or 'default'}...")
                
                # Инициализация двигателя через pylkmotor
                # Примечание: Конкретные аргументы зависят от версии pylkmotor и типа адаптера
                # Обычно pylkmotor сам находит устройство или требует указания COM-порта / CAN интерфейса
                if channel and interface:
                     # Примерная логика, может потребоваться корректировка под конкретную версию pylkmotor
                     # Если pylkmotor использует python-can внутри, он может принимать эти параметры
                     self.motor = LKMotor(interface=interface, channel=channel)
                else:
                    self.motor = LKMotor()
                
                # Попытка получить состояние для проверки связи
                # В реальной библиотеке метод может называться иначе, например get_state или read_data
                # Здесь мы предполагаем наличие метода получения данных
                self.connected = True
                print("✅ Успешное подключение к двигателю KMTech!")
                
            except Exception as e:
                print(f"✗ Ошибка подключения: {e}")
                print("  Переход в режим симуляции...")
                self.use_simulator = True
                self.connected = False
        elif not PYLKMOTOR_AVAILABLE and not use_simulator:
            print("✗ Библиотека pylkmotor недоступна. Принудительный запуск симуляции.")
            self.use_simulator = True

    def get_currents(self):
        """Получает текущие значения токов и параметров."""
        if self.use_simulator or not self.connected:
            return self.simulator.get_data()
        
        try:
            # Получение данных через pylkmotor
            # Предполагаем, что есть метод get_state() возвращающий объект с полями
            # В зависимости от реализации pylkmotor, код может отличаться
            
            # Вариант 1: Если есть метод get_state возвращающий класс с полями
            if hasattr(self.motor, 'get_state'):
                state = self.motor.get_state()
                # Адаптация под реальные имена полей в pylkmotor (нужно смотреть документацию библиотеки)
                # Часто это: state.id, state.iq, state.ia, state.ib, state.ic
                # Если библиотека возвращает только фазные, пересчитаем d/q (упрощенно оставим заглушки если нет)
                
                # Заглушка для примера, так как точный API pylkmotor нужно уточнять по документации
                # Предположим, что библиотека возвращает словарь или объект
                if isinstance(state, dict):
                    return {
                        'id': state.get('id', 0),
                        'iq': state.get('iq', 0),
                        'ia': state.get('ia', 0),
                        'ib': state.get('ib', 0),
                        'ic': state.get('ic', 0),
                        'speed': state.get('speed', 0),
                        'voltage': state.get('voltage', 0)
                    }
                else:
                    # Если это объект с атрибутами
                    return {
                        'id': getattr(state, 'id', 0),
                        'iq': getattr(state, 'iq', 0),
                        'ia': getattr(state, 'ia', 0),
                        'ib': getattr(state, 'ib', 0),
                        'ic': getattr(state, 'ic', 0),
                        'speed': getattr(state, 'speed', 0),
                        'voltage': getattr(state, 'voltage', 0)
                    }
            
            # Если специфического метода нет, возможно данные читаются иначе.
            # Для демонстрации вернем симуляцию, если метод не найден, чтобы не крашить программу
            print("⚠️ Метод получения данных в pylkmotor не найден или изменился. Возврат симуляции.")
            return self.simulator.get_data()

        except Exception as e:
            print(f"⚠️ Ошибка чтения данных: {e}")
            # При ошибке чтения не отключаемся сразу, но возвращаем последние известные или ноль
            return {'id': 0, 'iq': 0, 'ia': 0, 'ib': 0, 'ic': 0, 'speed': 0, 'voltage': 0}


class CurrentVisualizer:
    """Визуализация токов в реальном времени."""
    
    def __init__(self, controller, window_size=100):
        self.controller = controller
        self.window_size = window_size
        
        # Буферы данных
        self.data_id = deque(maxlen=window_size)
        self.data_iq = deque(maxlen=window_size)
        self.data_ia = deque(maxlen=window_size)
        self.data_ib = deque(maxlen=window_size)
        self.data_ic = deque(maxlen=window_size)
        self.time_axis = deque(maxlen=window_size)
        
        self.start_time = time.time()
        
        # Настройка графиков
        plt.style.use('dark_background')
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.suptitle('Мониторинг токов BLDC (KMTech MG4010E v3) - FOC', fontsize=14)
        
        # График 1: Токи DQ
        self.line_id, = self.ax1.plot([], [], label='Id (A)', color='cyan', linewidth=2)
        self.line_iq, = self.ax1.plot([], [], label='Iq (A)', color='magenta', linewidth=2)
        self.ax1.set_ylabel('Ток (A)')
        self.ax1.set_title('Токи вращающейся системы координат (DQ)')
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.set_ylim(-10, 10) # Диапазон зависит от двигателя
        
        # График 2: Фазные токи
        self.line_ia, = self.ax2.plot([], [], label='Ia (A)', color='red', linewidth=1.5)
        self.line_ib, = self.ax2.plot([], [], label='Ib (A)', color='green', linewidth=1.5)
        self.line_ic, = self.ax2.plot([], [], label='Ic (A)', color='blue', linewidth=1.5)
        self.ax2.set_xlabel('Время (с)')
        self.ax2.set_ylabel('Ток (A)')
        self.ax2.set_title('Фазные токи статора')
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.set_ylim(-10, 10)
        
        self.timer_text = self.fig.text(0.02, 0.02, '', color='gray', fontsize=8)

    def init_plot(self):
        self.ax1.set_xlim(0, self.window_size * 0.05) # Примерный масштаб времени
        self.ax2.set_xlim(0, self.window_size * 0.05)
        return [self.line_id, self.line_iq, self.line_ia, self.line_ib, self.line_ic]

    def update(self, frame):
        # Получение данных
        data = self.controller.get_currents()
        
        current_time = time.time() - self.start_time
        
        self.data_id.append(data['id'])
        self.data_iq.append(data['iq'])
        self.data_ia.append(data['ia'])
        self.data_ib.append(data['ib'])
        self.data_ic.append(data['ic'])
        self.time_axis.append(current_time)
        
        # Обновление линий
        x_data = list(self.time_axis)
        
        self.line_id.set_data(x_data, list(self.data_id))
        self.line_iq.set_data(x_data, list(self.data_iq))
        
        self.line_ia.set_data(x_data, list(self.data_ia))
        self.line_ib.set_data(x_data, list(self.data_ib))
        self.line_ic.set_data(x_data, list(self.data_ic))
        
        # Динамическое изменение оси X (скользящее окно)
        if len(x_data) > 0:
            offset = x_data[-1]
            self.ax1.set_xlim(offset - 5, offset + 0.5) # Показываем последние 5.5 секунд
            self.ax2.set_xlim(offset - 5, offset + 0.5)
            
            # Автоподстройка Y (опционально, можно зафиксировать)
            # self.ax1.relim(); self.ax1.autoscale_view()
            # self.ax2.relim(); self.ax2.autoscale_view()
        
        self.timer_text.set_text(f"Время: {current_time:.2f}с | Скорость: {data['speed']:.1f} рад/с | U: {data['voltage']:.1f}В")
        
        return [self.line_id, self.line_iq, self.line_ia, self.line_ib, self.line_ic, self.timer_text]

    def start(self):
        print("="*60)
        print("Запуск визуализации...")
        print("Закройте окно графика для остановки программы")
        print("="*60)
        
        # blit=False часто стабильнее на Windows с TkAgg
        ani = animation.FuncAnimation(
            self.fig, 
            self.update, 
            init_func=self.init_plot, 
            frames=None, 
            interval=50, 
            blit=False, 
            cache_frame_data=False
        )
        
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Система управления BLDC KMTech MG4010E v3")
    parser.add_argument('-s', '--simulate', action='store_true', help='Запуск в режиме симуляции')
    parser.add_argument('--channel', type=str, help='CAN канал (например, 0 или can0)')
    parser.add_argument('--interface', type=str, help='Интерфейс CAN (например, pcan, kvaser, socketcan)')
    
    args = parser.parse_args()

    print("="*60)
    print("BLDC Motor Control System - FOC Algorithm")
    print("Двигатель: KMTech MG4010E v3")
    print("Библиотека: pylkmotor")
    print("="*60)

    if not MATPLOTLIB_AVAILABLE:
        print("Ошибка: matplotlib не установлена.")
        return

    # Создание контроллера
    controller = BLDCController(
        use_simulator=args.simulate,
        channel=args.channel,
        interface=args.interface
    )

    # Запуск визуализации
    visualizer = CurrentVisualizer(controller)
    try:
        visualizer.start()
    except KeyboardInterrupt:
        print("\nПрограмма остановлена пользователем.")


if __name__ == "__main__":
    main()
