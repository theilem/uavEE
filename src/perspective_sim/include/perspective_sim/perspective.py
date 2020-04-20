from perspective_sim import util


class PerspectiveParams:
    def __init__(self, img_path):
        self.img_path = img_path
        self.base_img = util.load_image(img_path)


class Perspective:
    def __init__(self, params: PerspectiveParams):
        self.params = params
