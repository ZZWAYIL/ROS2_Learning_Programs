from demo_python_pkg.demo_person_node import Person

class Son(Person):
    def __init__(self, name: str,age : int ,book: str):
        super().__init__(name, age)
        self.book = book
    
def main():
    son = Son("ZZW", 10, "快速成为Python高手")
    print(son.eat("fish" ))
    print(f"My name is {son.name}, I am {son.age} years old, and I like reading {son.book}.")