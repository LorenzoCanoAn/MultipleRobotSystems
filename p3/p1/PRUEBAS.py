class pruebas:
    def __init__(self):
        self.fun = {"state1": self.state1,
                    "state2": self.state2,
                    "state3": self.state3}

    def state1(self):
        print("estado1")
    def state2(self):
        print("estado2")
    def state3(self):
        print("estado3")


if __name__ == "__main__":
    my_class = pruebas()

    my_class.fun["state1"]()