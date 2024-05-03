import time

from pynput import keyboard

from multinodes import Node


"""
    `actuator_command`, output (pipe)
        format:
            (type = 'vw', v, w)
            (type = 'vphi', v, phi)
"""
class KeyboardAckermannController(Node):
    def __init__(self, name,
        max_v = 8.0,
        delta_v = 4.0,
        decay_v = 0.3,
        max_phi = 0.48,
        delta_phi = 0.3,
        decay_phi = 0.3
    ):
        super().__init__(name)
        self.max_v = max_v
        self.delta_v = delta_v
        self.decay_v = decay_v
        self.max_phi = max_phi
        self.delta_phi = delta_phi
        self.decay_phi = decay_phi

    def run(self):
        v = 0.0
        phi = 0.0

        up_pressed = False
        down_pressed = False
        left_pressed = False
        right_pressed = False
        z_pressed = False
        space_pressed = False

        def on_press(key):
            nonlocal up_pressed, down_pressed, left_pressed, right_pressed, z_pressed, space_pressed
            if key == keyboard.Key.up:
                up_pressed = True
            elif key == keyboard.Key.down:
                down_pressed = True
            elif key == keyboard.Key.left:
                left_pressed = True
            elif key == keyboard.Key.right:
                right_pressed = True
            elif key == keyboard.KeyCode.from_char('z'):
                z_pressed = True
            elif key == keyboard.Key.space:
                space_pressed = True
        
        def on_release(key):
            nonlocal up_pressed, down_pressed, left_pressed, right_pressed, z_pressed, space_pressed
            if key == keyboard.Key.up:
                up_pressed = False
            elif key == keyboard.Key.down:
                down_pressed = False
            elif key == keyboard.Key.left:
                left_pressed = False
            elif key == keyboard.Key.right:
                right_pressed = False
            elif key == keyboard.KeyCode.from_char('z'):
                z_pressed = False
            elif key == keyboard.Key.space:
                space_pressed = False

        with keyboard.Listener(on_press = on_press, on_release = on_release) as listener:
            last_t = time.time()
            while True:
                # 检查接口
                if 'actuator_command' not in self.io:
                    time.sleep(0.1)
                    continue

                # 时间间隔
                current_t = time.time()
                dt = current_t - last_t
                last_t = current_t

                # 油门
                if up_pressed == down_pressed:
                    v = v * (1 - self.decay_v * dt)
                if up_pressed and not down_pressed:
                    if v < 0:
                        v = 0
                    else:
                        v = min(self.max_v, v + self.delta_v * dt)
                if down_pressed and not up_pressed:
                    if v > 0:
                        v = 0
                    else:
                        v = max(-self.max_v, v - self.delta_v * dt)

                # 转向
                if left_pressed == right_pressed:
                    phi = phi * (1 - self.decay_phi * dt)
                if left_pressed and not right_pressed:
                    phi = min(self.max_phi, phi + self.delta_phi * dt)
                if right_pressed and not left_pressed:
                    phi = max(-self.max_phi, phi - self.delta_phi * dt)
                
                # 回正
                if z_pressed:
                    phi = 0
                
                # 手刹
                if space_pressed:
                    v = 0

                self.io['actuator_command'].write(('vphi', v, phi)) # 已检查
                
                time.sleep(0.05)

