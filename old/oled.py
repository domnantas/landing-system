import time

import adafruit_ssd1306
import board
import busio
from PIL import Image, ImageDraw, ImageFont


def threshold(minValue): return lambda x: 255 if x > minValue else 0


class TrackerOled:
    def __init__(self):
        i2c = busio.I2C(board.SCL, board.SDA)
        self.oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)

        self.clearDisplay()
        self.showLogo()

    def clearDisplay(self):
        self.oled.fill(0)
        self.oled.show()

    def showLogo(self):
        logo = (
            Image.open("./Agai_melynas.png")
            .resize((self.oled.width, 48), Image.NEAREST)
            .convert("L").point(threshold(20), mode='1')
        )

        image = Image.new("1", (self.oled.width, self.oled.height))
        image.paste(logo, (0, 8))
        self.oled.image(image)
        self.oled.show()

    def drawPoint(self, x, y):
        image = Image.new("1", (self.oled.width, self.oled.height))
        draw = ImageDraw.Draw(image)
        draw.point([x, y], fill=255)
        draw.point([self.oled.width / 2, self.oled.height / 2], fill=255)
        draw.line([(self.oled.width / 2 - 4, self.oled.height / 2 - 4),
                   (self.oled.width / 2 + 4, self.oled.height / 2 + 4)], fill=255)
        draw.line([(self.oled.width / 2 - 4, self.oled.height / 2 + 4),
                   (self.oled.width / 2 + 4, self.oled.height / 2 - 4)], fill=255)
        self.oled.image(image)
        self.oled.show()

    def writeTextCenter(self, text):
        image = Image.new("1", (self.oled.width, self.oled.height))
        draw = ImageDraw.Draw(image)
        font = ImageFont.load_default()
        (font_width, font_height) = font.getsize(text)
        draw.text(
            (self.oled.width // 2 - font_width // 2,
             self.oled.height // 2 - font_height // 2),
            text,
            font=font,
            fill=255
        )
        self.oled.image(image)
        self.oled.show()
