from sympy import *
import random as r

a = Symbol('a')
b = Symbol('b')
c = Symbol('c')
d = Symbol('d')

x = Symbol('x')
y = Symbol('y')




poly = a*(x-y)**3 + b*(x-y)**2 + c*(x-y) + d
poly = poly.expand()
print collect(poly, x)

#result = solve(poly, x, simplified=False)
#print ""
#print result
