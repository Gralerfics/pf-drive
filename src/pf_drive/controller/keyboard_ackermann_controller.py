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
    def run(self):
        max_v = 8.0
        delta_v = 4.0
        decay_v = 0.3
        max_phi = 0.48
        delta_phi = 0.3
        decay_phi = 0.3

        v = 0.0
        phi = 0.0

        up_pressed = False
        down_pressed = False
        left_pressed = False
        right_pressed = False
        z_pressed = False # 回正
        space_pressed = False # 手刹

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
            while not self.is_shutdown():
                current_t = time.time()
                dt = current_t - last_t
                last_t = current_t

                if up_pressed == down_pressed:
                    v = v * (1 - decay_v * dt)
                if up_pressed and not down_pressed:
                    if v < 0:
                        v = 0
                    else:
                        v = min(max_v, v + delta_v * dt)
                if down_pressed and not up_pressed:
                    if v > 0:
                        v = 0
                    else:
                        v = max(-max_v, v - delta_v * dt)

                if left_pressed == right_pressed:
                    # phi = 0
                    phi = phi * (1 - decay_phi * dt)
                if left_pressed and not right_pressed:
                    # if phi < 0:
                    #     phi = 0
                    # else:
                    phi = min(max_phi, phi + delta_phi * dt)
                if right_pressed and not left_pressed:
                    # if phi > 0:
                    #     phi = 0
                    # else:
                    phi = max(-max_phi, phi - delta_phi * dt)
                
                if z_pressed:
                    phi = 0
                
                if space_pressed:
                    v = 0

                self.io['actuator_command'].write(('vphi', v, phi))
                time.sleep(0.05)

