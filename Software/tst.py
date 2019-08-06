
class A(object):
    val = 1
    def foo(self):
        self.val += 3

    def op(self):
        A.val += 5

    def sop():
        print('Sop')


a = A()
b = A()

print(a.val)
print(b.val)

a.foo()
b.op()

c = A()
A.sop()
print(a.val)
print(b.val)
print(c.val)
