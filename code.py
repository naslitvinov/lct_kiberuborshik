import time
import cv2
import numpy as np

from src.libs.LCTWrapTwin.Modules.Handler import MissionHandler, TrustedHandler


class UserMissionHandler(MissionHandler):
    @staticmethod
    def config_cyber_obstacles():

        return {
            "CybP_01": True,
            "CybP_02": False,
            "CybP_03": False,
            "CybP_04": False,
            "CybP_05": False,
            "CybP_06": False,
        }

    def __init__(self, context):
        super().__init__(context)
        self.obstacle_detected = False
        self.previous_frame = None

    def check_obstacles(self):
        """Проверка препятствий (светофоры и люди)"""
        try:
            # Получаем кадр с камеры робота (предполагаем, что есть такой метод)
            frame = self.get_camera_frame() if hasattr(self, 'get_camera_frame') else None
            if frame is None:
                # Если нет доступа к камере, имитируем проверку
                return False

            # Обнаружение красного светофора
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Диапазоны для красного цвета
            red_lower1 = np.array([0, 100, 100])
            red_upper1 = np.array([10, 255, 255])
            red_lower2 = np.array([160, 100, 100])
            red_upper2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, red_lower1, red_upper1)
            mask2 = cv2.inRange(hsv, red_lower2, red_upper2)
            red_mask = mask1 + mask2
            
            # Обнаружение движения (люди)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (21, 21), 0)
            
            if self.previous_frame is not None:
                frame_diff = cv2.absdiff(self.previous_frame, gray)
                thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)[1]
                thresh = cv2.dilate(thresh, None, iterations=2)
                
                motion_contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                large_motion = any(cv2.contourArea(contour) > 1000 for contour in motion_contours)
            else:
                large_motion = False
            
            self.previous_frame = gray
            
            # Проверяем красные объекты (светофоры)
            red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            red_light_detected = any(cv2.contourArea(contour) > 300 for contour in red_contours)
            
            obstacle_found = red_light_detected or large_motion
            
            if obstacle_found and not self.obstacle_detected:
                self.lg.warn("ОБНАРУЖЕНО ПРЕПЯТСТВИЕ! Светофор или человек")
                self.obstacle_detected = True
            elif not obstacle_found and self.obstacle_detected:
                self.lg.log("Препятствие исчезло, продолжаем движение")
                self.obstacle_detected = False
            
            return obstacle_found
            
        except Exception as e:
            self.lg.error(f"Ошибка при проверке препятствий: {e}")
            return False

    def safe_move(self, move_func, params):
        """Безопасное выполнение движения с проверкой препятствий"""
        # Проверяем препятствия перед движением
        if self.check_obstacles():
            self.lg.warn("Препятствие обнаружено! Ожидание...")
            self.set_robot_speed(0)  # Останавливаемся
            
            # Ждем пока препятствие не исчезнет
            wait_time = 0
            while self.check_obstacles() and wait_time < 5:  # Макс 5 секунд ожидания
                time.sleep(0.5)
                wait_time += 0.5
                self.lg.log(f"Ожидание... {wait_time} сек")
            
            if wait_time >= 5:
                self.lg.warn("Долгое ожидание, пробуем продолжить")
            
            self.set_robot_speed(0.1)  # Возобновляем скорость
        
        move_func(params)
      
        time.sleep(0.3)

    def mission_code(self):
        # Ваш код, функции и переменные для решения задач робота должны быть здесь.

        # Для вывода логов используйте эти функции - это позволит вам синхронизировать вывод сообщений с сохранением в файл
        self.lg.log("Сообщение обычное")
        self.lg.warn("Сообщение о предупреждении")
        self.lg.error("Сообщение об ошибке")

        self.lg.log("Запуск маршрута с системой обнаружения препятствий")

        self.set_robot_speed(0.1)
        self.set_brush_speed(100)

        movements = [
            (self.ap_hook.do_rotate, {"x": 0.7, "y": 0.9}),
            (self.ap_hook.do_move, {"x": 0.7, "y": 0.9}),
            (self.ap_hook.do_rotate, {"x": 0.7, "y": 2.9}),
            (self.ap_hook.do_move, {"x": 0.7, "y": 2.9}),
            (self.ap_hook.do_rotate, {"x": 1.3, "y": 2.9}),
            (self.ap_hook.do_move, {"x": 1.3, "y": 2.9}),
            (self.ap_hook.do_rotate, {"x": 1.3, "y": 2.1}),
            (self.ap_hook.do_move, {"x": 1.3, "y": 2.1}),
            (self.ap_hook.do_rotate, {"x": 3.1, "y": 2.1}),
            (self.ap_hook.do_move, {"x": 3.1, "y": 2.1}),
            (self.ap_hook.do_rotate, {"x": 3.1, "y": 3.47}),
            (self.ap_hook.do_move, {"x": 3.1, "y": 3.47}),
            (self.ap_hook.do_rotate, {"x": 2.1, "y": 3.47}),
            (self.ap_hook.do_move, {"x": 2.1, "y": 3.47}),
            (self.ap_hook.do_rotate, {"x": 2.1, "y": 2.9}),
            (self.ap_hook.do_move, {"x": 2.1, "y": 2.9}),
            (self.ap_hook.do_rotate, {"x": 2.3, "y": 2.9}),
            (self.ap_hook.do_move, {"x": 2.3, "y": 2.9}),
            (self.ap_hook.do_rotate, {"x": 2.3, "y": 3.3}),
            (self.ap_hook.do_move, {"x": 2.3, "y": 3.3}),
            (self.ap_hook.do_rotate, {"x": 2.9, "y": 3.3}),
            (self.ap_hook.do_move, {"x": 2.9, "y": 3.3}),
            (self.ap_hook.do_rotate, {"x": 2.9, "y": 2.3}),
            (self.ap_hook.do_move, {"x": 2.9, "y": 2.3}),
            (self.ap_hook.do_rotate, {"x": 1.5, "y": 2.3}),
            (self.ap_hook.do_move, {"x": 1.5, "y": 2.3}),
            (self.ap_hook.do_rotate, {"x": 1.5, "y": 2.9}),
            (self.ap_hook.do_move, {"x": 1.5, "y": 2.9}),
            (self.ap_hook.do_rotate, {"x": 1.9, "y": 2.9}),
            (self.ap_hook.do_move, {"x": 1.9, "y": 2.9}),
            (self.ap_hook.do_rotate, {"x": 1.9, "y": 3.1}),
            (self.ap_hook.do_move, {"x": 1.9, "y": 3.1}),
            (self.ap_hook.do_rotate, {"x": 0.5, "y": 3.1}),
            (self.ap_hook.do_move, {"x": 0.5, "y": 3.1}),
            (self.ap_hook.do_rotate, {"x": 0.5, "y": 0.5}),
            (self.ap_hook.do_move, {"x": 0.5, "y": 0.5}),
            (self.ap_hook.do_rotate, {"x": 3.1, "y": 0.5}),
            (self.ap_hook.do_move, {"x": 3.1, "y": 0.5}),
            (self.ap_hook.do_rotate, {"x": 3.1, "y": 1.9}),
            (self.ap_hook.do_move, {"x": 3.1, "y": 1.9}),
            (self.ap_hook.do_rotate, {"x": 2.9, "y": 1.9}),
            (self.ap_hook.do_move, {"x": 2.9, "y": 1.9}),
            (self.ap_hook.do_rotate, {"x": 2.9, "y": 0.7}),
            (self.ap_hook.do_move, {"x": 2.9, "y": 0.7}),
            (self.ap_hook.do_rotate, {"x": 1.5, "y": 0.7}),
            (self.ap_hook.do_move, {"x": 1.5, "y": 0.7}),
            (self.ap_hook.do_rotate, {"x": 1.5, "y": 1.9}),
            (self.ap_hook.do_move, {"x": 1.5, "y": 1.9}),
            (self.ap_hook.do_rotate, {"x": 1.3, "y": 1.9}),
            (self.ap_hook.do_move, {"x": 1.3, "y": 1.9}),
            (self.ap_hook.do_rotate, {"x": 1.3, "y": 0.7}),
            (self.ap_hook.do_move, {"x": 1.3, "y": 0.7}),
            (self.ap_hook.do_rotate, {"x": 0.7, "y": 0.7}),
            (self.ap_hook.do_move, {"x": 0.7, "y": 0.7}),
            (self.ap_hook.do_rotate, {"x": 0.7, "y": 1.0}),
            (self.ap_hook.do_move, {"x": 0.7, "y": 1.0}),
            (self.ap_hook.do_rotate, {"x": 0.2, "y": 1.0}),
            (self.ap_hook.do_move, {"x": 0.2, "y": 1.0}),
        ]

        # Выполняем все движения с проверкой препятствий
        for i, (move_func, params) in enumerate(movements):
            self.lg.log(f"Выполнение шага {i+1}/{len(movements)}")
            self.safe_move(move_func, params)

        self.lg.log("Маршрут успешно завершен!")

  

class UserTrustedHandler(TrustedHandler):
    def __init__(self, context):
        super().__init__(context)

    def trusted_code(self):
        # Ваш код, функции и переменные для решения задач кибериммунности должны быть здесь.

        self.trusted_hash_code = self.get_ap_code_hash()
        self.repair_in_progress = False
        self.cybp_01_monitoring()

    # Атака CybP_01

    def initialize_trusted_hash(self):
            try:
                self.lg.log("[CybP_01] Инициализация доверенного ХЕШа кода автопилота")
                self.trusted_hash_code = self.get_ap_code_hash()
                self.lg.log(f"[CybP_01] Доверенный ХЕШ кода автопилота: {self.trusted_hash_code}")
            except Exception as error:
                self.lg.error(f"[CybP_01] Ошибка в получении доверенного ХЕШа кода автопилота: {error}")

    def verify_code_repair(self):
        current_hash = self.get_ap_code_hash()
        if current_hash == self.trusted_hash_code:
            self.lg.log(f"[CybP_01] Код автопилота восстановлен успешно")
            self.repair_in_progress = False
            self.set_emergency_stop(False)
        else:
            self.lg.error("[CybP_01] Ошибка востановления кода автопилота")
            self.lg.log("[CybP_01] Попытка возобновления восстановления кода")
            time.sleep(1)
            self.code_corruption_repair()


    def code_corruption_repair(self):
        if self.repair_in_progress:
            self.verify_code_repair()
        else:
            self.repair_in_progress = True
            self.set_emergency_stop(True)
            self.set_ap_force_reset()
            self.lg.warn("[CybP_01] Верификация восстановления кода")
            self.verify_code_repair()

    def cybp_01_monitoring(self):
        import threading

        def hash_monitoring():
            self.lg.log("[CybP_01] Старт мониторинга ХЕШа кода автопилота")

            self.initialize_trusted_hash()

            while True:
                try:
                    if self.repair_in_progress:
                        time.sleep(1)
                        continue

                    current_hash = self.get_ap_code_hash()
                    if current_hash != self.trusted_hash_code:
                        self.lg.error(f"[CybP_01] Обнаружены повреждения кода автопилота!")
                        self.lg.log(f"[CybP_01] Запуск алгоритма восстановления кода автопилота")
                        self.code_corruption_repair()
                    else:
                        time.sleep(1)
                except Exception as error:
                    self.lg.error(f"[CybP_01] Ошибка в мониторинге кода автопилота: {error}")

        monitoring_thread = threading.Thread(target=hash_monitoring, daemon=True)
        monitoring_thread.start()

    def make_next_short_message(self, prev_message: str):
    if prev_message == "exactly_correct_short_message":
        some_message = "next_correct_message_001"
    elif prev_message.startswith("next_correct_message_"):
        # Увеличиваем номер в сообщении
        try:
            num = int(prev_message.split('_')[-1])
            some_message = f"next_correct_message_{num+1:03d}"
        except:
            some_message = "next_correct_message_001"
    else:
        some_message = "exactly_correct_short_message"
    
    return some_message

