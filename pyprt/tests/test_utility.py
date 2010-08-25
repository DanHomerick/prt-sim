import unittest
import timeit

import numpy

import pyprt.shared.utility as utility

class TestUtility(unittest.TestCase):
    def check_roots(self, coeffs, correct=None, threshold=None):
        if correct is None:
            correct = numpy.roots(coeffs)
            correct = [x.real for x in correct if abs(x.imag) < 1E-8] # just real roots

        if len(coeffs) == 3:
            if threshold is None:
                to_check = utility.quadratic_roots(*coeffs)
            else:
                to_check = utility.quadratic_roots(coeffs[0], coeffs[1], coeffs[2], threshold)
        elif len(coeffs) == 4:
            if threshold is None:
                to_check = utility.cubic_roots(*coeffs)
            else:
                to_check = utility.cubic_roots(coeffs[0], coeffs[1], coeffs[2], coeffs[3], threshold)
        else:
            raise Exception("Unexpected case.")

        to_check.sort()
        correct.sort()
        if len(correct) == 1 and len(to_check) == 3:
            # numpy.roots is sometimes missing a tangent root. If two more roots are
            # found than numpy.roots reports, just check that they're close to each
            # other.
            if abs(correct[0] - to_check[0]) < abs(correct[0] - to_check[2]):
                self.assertAlmostEqual(to_check[0], correct[0], 5)
                self.assertAlmostEqual(to_check[1], to_check[2], 5)
            else:
                self.assertAlmostEqual(to_check[2], correct[0], 5)
                self.assertAlmostEqual(to_check[0], to_check[1], 5)
        else:
            for check, corr in zip(to_check, correct):
                self.assertAlmostEqual(check, corr, 5)

    def test_linear_root(self):
        self.assertEqual(utility.linear_root(5, 10), [-2])
        self.assertEqual(utility.linear_root(0, 10), [])

    def test_quadratic_roots(self):
        self.check_roots([1,0,0], correct=[0])
        self.check_roots([1,0,-4])
        self.check_roots([1,-5,0])
        self.check_roots([1,4,4], correct=[-2])
        self.check_roots([1,0,1], correct=[])

        # No real roots, but within threshold distance
        self.check_roots([1,0,1E-5], correct=[0], threshold=1.1E-5)
        self.check_roots([-1,0,-1E-5], correct=[0], threshold=1.1E-5)
        self.check_roots([2,-12,17], correct=[3], threshold=1.0)

        # Normally two real roots, but within threshold distance
        self.check_roots([1,0,-1E-5], correct=[0], threshold=1.1E-5)
        self.check_roots([-1,0,1E-5], correct=[0], threshold=1.1E-5)
        self.check_roots([2,-12,19], correct=[3], threshold=1.0)

    def test_cubic_roots(self):
        self.assertRaises(ValueError, utility.cubic_roots, 0,1,1,1)
        self.check_roots([-1000,-1000,-250,0], threshold=1E-8)
        self.check_roots([-1000,-600,150,100], threshold=1E-8)
        self.check_roots([-1000,0,0,0], threshold=1E-8)
##        self.check_roots([0.41666666666666669, -77666.624999984706, 4825683711.1106014, -99945217909163.203], correct=56855.3)

##        # Validating a routine when you don't have a good source of 'correct'
##        # results is tough. Calling check_roots never completes
##        # all the checks because it winds up hitting an imprecise result
##        # (accurate to fewer than 5 places) from numpy.roots.
##        A = numpy.arange(-1000, 1000, 50)
##        B = numpy.arange(-1000, 1000, 50)
##        C = numpy.arange(-1000, 1000, 50)
##        D = numpy.arange(-1000, 1000, 50)
##        for a in A:
##            if a == 0:
##                continue
##            for b in B:
##                for c in C:
##                    for d in D:
##                        utility.cubic_roots(a,b,c,d, threshold=1E-8)
####                        coeffs = [a,b,c,d]
####                        self.check_roots(coeffs, threshold=1E-8)


def quadratic_roots_timing():
    print timeit.repeat('utility.quadratic_roots(2,-12,17,1.0)', setup='import pyprt.shared.utility as utility', number=1000)
    print timeit.repeat('numpy.roots([2,-12,17])', setup='import numpy', number=1000)
    # Results:
    # [0.0013951003835029165, 0.0013373632431678789, 0.0013852636855199102]
    # [0.15358806389405791, 0.15072387405091905, 0.14334121837341218]
    # For quadratic polynomials, utility.quadratic_roots is 112 times faster.

def cubic_roots_timing():
    print timeit.repeat('utility.cubic_roots(-500,-960, -210, 5)', setup='import pyprt.shared.utility as utility', number=1000)
    print timeit.repeat('numpy.roots([-500,-960, -210, 5])', setup='import numpy', number=1000)
    # Results:
    # [0.0069695143622205968, 0.005970875675684939, 0.0058070732701417938]
    # [0.15618452447905057, 0.15583510785939336, 0.16456239738292511]
    # For cubic polynomials, utility.cubic_roots is 28 times faster (takes 4% as much time)


if __name__ == '__main__':
    quadratic_roots_timing()
    cubic_roots_timing()
    unittest.main()
