#!/usr/bin/env python3

class Person:
    def __init__(self, name: str, age: int):
        self.name = name
        self.age = age

    def eat(self, food: str):
        return f"{self.name} is eating {food}."

    def greet(self) -> str:
        return f"Hello, my name is {self.name} and I am {self.age} years old."
    
def main():
    person = Person("Alice", 30)
    person1 = Person("小张", 21)
    print(person.greet())
    print(person.eat("apple"))
    print(person1.eat("banana"))

    
# if __name__ == "__main__":
#     main()