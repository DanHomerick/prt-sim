from __future__ import division
from sympy import __version__
from sympy import *

"""Much of the algerbra and calculus done while developing
trajectory_solver.py is in this file. Aside from debugging or further development
on trajector_solver, this script has no use and does not need to be ran."""


### Key
# *x == maximum
# *n == minimum
# *t == top (highest value reached in trajectory. Like maximum, but discovered rather than given.)
# *b == bottom (lowest value)
# j == jerk
# a == acceleration
# v == velocity
# q == displacement
# *01 == delta from point 0 to point 1
###

def prob1():
    ### Problem 1: v_top is < v_max (or v_max unspecified). a_max is reached.
    #        Known that the spline should have an acceleration profile of:
    #          h0  h1    h2   h3  h4
    #             ____
    #            /    \
    #           /      \
    #          /        \
    #                    \          /
    #                     \        /
    #                      \______/
    #
    #         t0 t1  t2    t3    t4 t5
    #
    #
    #        where h's are the timespans of each polynomial, and t's are the knot times.
    #        Note that the enpoints are not assumed to be 0 in either acceleration
    #        or velocity, nor are they assumed to be equal.

    jx, ax = symbols(['jx', 'ax'], real=True, positive=True)
    jn, an = symbols(['jn', 'an'], real=True, negative=True)
    vt, vb = symbols(['vt', 'vb'], real=True)
    a0, a5, v0, v5, q0, q5 = symbols(['a0', 'a5', 'v0', 'v5', 'q0', 'q5'], real=True)
    h0, h1, h2, h3, h4, h5 = symbols(['h0', 'h1', 'h2', 'h3', 'h4', 'h5'], real=True, nonnegative=True)

    ###
    # The approach I'm taking is to get everything to be a function of h1.
    # Segments 0, 2, and 4 are all constant. Adjusting the lengths of segments
    # 1 and 3 (and their ratio) determines the final position and velocity of the
    # trajectory.
    ### Get h1 as a function of h3
    h0 = (ax - a0)/jx # note that all changes in acceleration are done at max or min jerk
    v01 = (jx*h0**2)/2 + a0*h0              # known

    h1 = Symbol('h1')
    v12 = ax*h1                             # function of h1

    h2 = (an-ax)/jn                         # known
    v23 = (jn*h2**2)/2 + ax*h2              # known

    h3 = Symbol('h3')
    v34 = an*h3                             # function of h3

    h4 = (a5-an)/jx                         # known
    v45 = (jx*h4**2)/2 + an*h4              # known

    v4 = Symbol('v4')                       # known by v4 = v5 - v45
    v1 = Symbol('v1')                       # known by v1 = v0 + v01
    # Use v12 = v4 - v1 - v23 - v34 and v12 = ax*h1 to get:
    h1 = (v4 - v1 - v23 - v34)/ax           # function of h3
    pprint(together(collect(powsimp(apart(h1, h3)), h3)))

    #----
    # Simplify the formula for h1 to avoid carrying around so much symbology
    alpha = Symbol('alpha')
    h1 = alpha - an/ax*h3
    v12 = ax*h1  # redo v12 using the new formula

    ### Solve for h3
    q4 = Symbol('q4')                       # known by q4 = q5 - q45
    q1 = Symbol('q1')                       # known by q1 = q0 + q01

    q12 = ax*h1**2/2 + v1*h1

    v2 = v1 + v12
    q23 = jn*h2**3/6 + ax*h2**2/2 + v2*h2

    v3 = v2 + v23
    q34 = an*h3**2/2 + v3*h3

    # Use 0 = q4 - q1 - q12 - q23 - q34, and get in standard polynomial form:
    poly = q4 - q1 - q12 - q23 - q34
    pprint(collect(powsimp(apart(poly, h3), deep=True), h3))
    print ""
    print collect(powsimp(apart(poly, h3), deep=True), h3)


def prob2():
    """Problem 2: v_top is < v_max (or v_max unspecified) and a_top < a_max (or a_max unspecified).
    UNABLE TO SOLVE EXACTLY. REQUIRES root of 3rd degree poly.
    Assumed that the spline should have an acceleration profile of:

      h0   h2  h3

        /\
       /  \
      /    \
            \    /
             \  /
              \/

     t0 t1    t2 t3

    where h's are the timespans of each polynomial, and t's are the knot times.
    """
    # known values
    jx, jn, a0, a3, v0, v3, q0, q3 = symbols(['jx', 'jn', 'a0', 'a3', 'v0', 'v3', 'q0', 'q3'], real=True)

    at, ab = symbols(['at', 'ab'])
    h0 = (at-a0)/jx
    h1 = (ab-at)/jn
    h2 = (a3-at)/jx

    v01 = (at**2 - a0**2)/(2*jx)
    v12 = (jn*h1**2)/2 + at*h1
    v23 = (a3**2 - ab**2)/(2*jx)

    # Everything so far is in terms of at and ab. Get ab in terms of at.
    # v3 - v0 = v01 + v12 + v23
    # 0 = v3 - v0 - v01 - v12 - v23

#      2                                            2        2        2        2
#    ab *(jn - jx) - 2*jn*jx*v0 + 2*jn*jx*v3 + jn*a0  + jx*at  - jn*a3  - jn*at
#    ---------------------------------------------------------------------------
#                                      2*jn*jx

#    ab = solve(v3 - v0 - v01 - v12 - v23, ab)
#    ab_1 = ab[0]
#    ab_2 = ab[1]
#    pprint(ab)

    q01 = jx*h0**3/6 + a0*h0**2/2 + v0*h0

    v1 = v0 + v12
    v2 = v3 - v23
    q23 = jx*h2**3/6 + ab*h2**2/2 + v2*h2

    q12 = jn*h1**3/6 + at*h1**2/2 + v1*h1 # from left
    q21 = jn*h1**3/6 - ab*h1**2/2 + v2*h1 # from right


    ab = solve(v3 - v0 - v01 - v12 - v23, ab)[0] # arbitrarily using first solution

    ### try revaluating all this crap, now that I know ab
    h0 = (at-a0)/jx
    h1 = (ab-at)/jn
    h2 = (a3-at)/jx

    v01 = (at**2 - a0**2)/(2*jx)
    v12 = (jn*h1**2)/2 + at*h1
    v23 = (a3**2 - ab**2)/(2*jx)

    q01 = jx*h0**3/6 + a0*h0**2/2 + v0*h0

    v1 = v0 + v12
    v2 = v3 - v23
    q23 = jx*h2**3/6 + ab*h2**2/2 + v2*h2

    q12 = jn*h1**3/6 + at*h1**2/2 + v1*h1 # from left
    q21 = jn*h1**3/6 - ab*h1**2/2 + v2*h1 # from right
    ### done reevaluating

    pprint(h1)

#    pprint(q3 - q0 - q01 - q12 - q23)
#    print '--'
#    pprint(q3 - q0 - q01 + q21 - q23)
#    print '--'
#    pprint(v3 - v0 - v01 - v12 - v23)


def prob3():
    """Solving for target_position where a_max is not reached.
    Assuming an acceleration profile of:

     /\
    /  \
    0 1 2
    """

    at, a0, a2, v0, v2, jx, jn, ax = symbols(['at', 'a0', 'a2', 'v0', 'v2', 'jx', 'jn', 'ax'], real=True)

    h0 = (at-a0)/jx
    v01 = jx*h0**2/2 + a0*h0

    h1 = (a2-at)/jn
    v12 = jn*h1**2/2 + at*h1

    ans = v2 - v0 - v01 - v12
    pprint(collect(powsimp(apart(ans, at), deep=True), at))
    print collect(powsimp(apart(ans, at), deep=True), at)


def prob4():
    """In the Google Transit Feed controller, we need to find what velocity the
    vehicles should travel at in order to arrive at their next stop on time, or
    a litte early. Accuracy is not of great concern in this case, so the
    problem is simplified by assuming infinite jerk. Thus the acceleration
    profile looks like:
        ______
       |      |_____
                    |______|

      t0     t1    t2      t3

    """
    q0, q3, t0, t3, vx, ax = symbols(['q0', 'q3', 't0', 't3', 'vx', 'ax'], real=True, nonnegative=True)
    an = symbols(['an'], real=True, nonpositive=True)
    h0 = vx/ax
    v01 = ax*h0
    q01 = ax*h0**2/2  # assume v0 = 0

    h2 = -vx/an
    v23 = an*h2
    q23 = an*h2**2/2 + vx*h2

    q12 = q3 - q0 - q01 - q23
    h1 = t3 - t0 - h0 - h2

    ans = q12/h1
    pprint(ans)
    # From here, use the fact that ans is vx and the quadratic equation to solve for vx:
    # 0 = vx**2*(1/(2*an) - 1/(2*ax)) + vx*(t3-t0) - (q3-q0)

def prob5():
    """Finding the coefficients of a cubic polynomial from the accel, vel, and
    pos at two timepoints, as well as the jerk (constant)"""
    t, c, j, t1, t2, a1, a2, v1, v2, q1, q2 = symbols(['t','c','j','t1','t2','a1','a2','v1','v2','q1','q2'], real=True)
    ca = (t1*a2 - t2*a1)/(t1 - t2)
    a = j*t + ca

    v = integrate(a, t)
    cv = (t1**2*t2**2/(t1**2*t2**2 - t2**4))*(v2 - ca*t2) - (t1**2*t2**2/(t1**4 - t1**2*t2**2))*(v1 - ca*t1)
    v += cv

    q = integrate(v, t)
    cq = (t1**3*t2**3/(t1**3*t2**3 - t2**6))*(-ca*t2**2/2 - cv*t2 + q2) - (t1**3*t2**3/(t1**6 - t1**3*t2**3))*(-ca*t1**2/2 - cv*t1 + q1)
    q += cq

    ans = collect(powsimp(together(powsimp(q, deep=True), deep=True), deep=True), t, t**2, t**3)

    pprint(ans)
    print ""
    print ans
#    ans = collect(powsimp(ans, deep=True), t, t**2, t**3)
#    pprint(ans)
#    print ans

def prob6():
    """Finding a trajectory that targets a final postition, velocity, accel, and TIME.
    Vary max velocity to achieve this."""

    ax, an, jx, jn, vx, a0, a7, v0, v7, q0, q7, t0, t7 = symbols(['ax','an','jx','jn','vx','a0','a7','v0','v7','q0','q7', 't0', 't7'], real=True)

    # Accel
    t01 = (ax-a0)/jx
    v01 = powsimp(expand(jx*t01**2/2 + a0*t01))
    v1 =  v0 + v01
    q01 = powsimp(expand(jx*t01**3/6 + a0*t01**2/2 + v0*t01))

    a2 = ax
    t23 = -a2/jn
    v23 = powsimp(expand(jn*t23**2/2 + a2*t23))
    v2 =  vx - v23
    q23 = powsimp(expand(jn*t23**3/6 + a2*t23**2/2 + v2*t23))

    v12 = powsimp(expand(v2 - v1))
    a1 = ax
    t12 = v12/a1
    q12 = powsimp(expand(a1*t12**2/2 + v1*t12)) # jerk is 0

    q03 = powsimp(expand(q01 + q12 + q23))
    t03 = powsimp(expand(t01 + t12 + t23))

    # Decel
    a6 = an
    a67 = a7 - a6
    t67 = powsimp(expand(a67/jx))
    v67 = powsimp(expand(jx*t67**2/2 + a6*t67))
    v6 = v7 - v67
    q67 = powsimp(expand(jx*t67**3/6 + a6*t67**2/2 + v6*t67))

    a4 = 0
    a5 = an
    a45 = a5 - a4
    t45 = a45/jn
    v45 = powsimp(expand(jn*t45**2/2 + a4*t45)) # a4 is 0
    v5 = vx + v45
    q45 = powsimp(expand(jn*t45**3/6 + a4*t45**2/2 + vx*t45)) # a4 is 0

    v56 = powsimp(expand(v6 - v5))
    t56 = v56/an
    q56 = powsimp(expand(an*t56**2/2 + v5*t56)) # 0 jerk

    q47 = powsimp(expand(q45 + q56 + q67))
    t47 = powsimp(expand(t45 + t56 + t67))

    # Cruise
    q34 = powsimp(expand(q7 - q0 - q03 - q47))
    t34 = powsimp(expand(t7 - t0 - t03 - t47))
#    answer = solve(vx*t34/q34 - 1, vx)
#    pprint(answer)

#    pprint(together(expand(vx*t34/q34 - 1)))
    print "+++++++++++++++"
    print "S1:"
    s1 = together(expand(vx*t34/q34 - 1))
    print s1
    print "++++++++++++++"
    print "S2:"
    s2 = apart(s1, jx)
    pprint(s2)
    print "++++++++++++++"
    print "S3:"
    s3 = apart(s2, jn)
    pprint(s3)
    print "++++++++++++++"
    print "S4:"
    s4 = apart(s3, ax)
    pprint(s4)
    print "++++++++++++++"
    print "S5:"
    s5 = apart(s4, ax)
    pprint(s5)

if __name__ == '__main__':
    prob6()