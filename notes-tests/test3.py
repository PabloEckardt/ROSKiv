'''
Rotation Example
================

This example rotates a button using PushMatrix and PopMatrix. You should see
a static button with the words 'hello world' rotated at a 45 degree angle.
'''


from kivy.app import App
from kivy.lang import Builder

kv = '''
FloatLayout:

    Button:
        text: 'hello world'
        size_hint: .015, .25
        pos_hint: {'center_x': .5, 'center_y': .5}
        canvas.before:
            PushMatrix
            Rotate:
                angle: 90
                origin: self.center_x, self.y
		        axis: 0, 0, 1
        canvas.after:
            PopMatrix
'''


class RotationApp(App):
    def build(self):
        return Builder.load_string(kv)


RotationApp().run()
