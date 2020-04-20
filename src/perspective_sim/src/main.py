#!/usr/bin/env python
from perspective_sim import util
from perspective_sim import perspective

from matplotlib import pyplot as plt

if __name__ == "__main__":
    print(util.getName())
    params = perspective.PerspectiveParams("src/perspective_sim/res/img.png")
    plt.imshow(params.base_img)
    plt.show()
