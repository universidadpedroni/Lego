from pybricks.hubs import TechnicHub
from pybricks.parameters import Color
from pybricks.tools import wait

hub = TechnicHub()

if __name__ == "__main__":
    for i in range(3):
        hub.light.on(Color.YELLOW)
        wait(1000)
        hub.light.off()
        wait(1000)
  
    print("Terminado")
    