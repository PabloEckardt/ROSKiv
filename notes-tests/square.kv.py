#:kivy 1.0.9

<square>:
    size: 20,20
    canvas:
        Rectangle:
            pos: self.center_x-5, 0
            size: 2, 2

<looper>:
    sq: square
    canvas:
        Rectangle:
            pos: self.center_x - 5, 0
            size: 10, self.height