from csp import Constraint, Variable
import util

class TableConstraint(Constraint):
    '''General type of constraint that can be use to implement any type of
       constraint. But might require a lot of space to do so.

       A table constraint explicitly stores the set of satisfying
       tuples of assignments.'''

    def __init__(self, name, scope, satisfyingAssignments):
        '''Init by specifying a name and a set variables the constraint is over.
           Along with a list of satisfying assignments.
           Each satisfying assignment is itself a list, of length equal to
           the number of variables in the constraints scope.
           If sa is a single satisfying assignment, e.g, sa=satisfyingAssignments[0]
           then sa[i] is the value that will be assigned to the variable scope[i].


           Example, say you want to specify a constraint alldiff(A,B,C,D) for
           three variables A, B, C each with domain [1,2,3,4]
           Then you would create this constraint using the call
           c = TableConstraint('example', [A,B,C,D],
                               [[1, 2, 3, 4], [1, 2, 4, 3], [1, 3, 2, 4],
                                [1, 3, 4, 2], [1, 4, 2, 3], [1, 4, 3, 2],
                                [2, 1, 3, 4], [2, 1, 4, 3], [2, 3, 1, 4],
                                [2, 3, 4, 1], [2, 4, 1, 3], [2, 4, 3, 1],
                                [3, 1, 2, 4], [3, 1, 4, 2], [3, 2, 1, 4],
                                [3, 2, 4, 1], [3, 4, 1, 2], [3, 4, 2, 1],
                                [4, 1, 2, 3], [4, 1, 3, 2], [4, 2, 1, 3],
                                [4, 2, 3, 1], [4, 3, 1, 2], [4, 3, 2, 1]])
          as these are the only assignments to A,B,C respectively that
          satisfy alldiff(A,B,C,D)
        '''

        Constraint.__init__(self,name, scope)
        self._name = "TableCnstr_" + name
        self.satAssignments = satisfyingAssignments

    def check(self):
        '''check if current variable assignments are in the satisfying set'''
        assignments = []
        for v in self.scope():
            if v.isAssigned():
                assignments.append(v.getValue())
            else:
                return True
        return assignments in self.satAssignments

    def hasSupport(self, var,val):
        '''check if var=val has an extension to an assignment of all variables in
           constraint's scope that satisfies the constraint. Important only to
           examine values in the variable's current domain as possible extensions'''
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in
        vindex = self.scope().index(var)
        found = False
        for assignment in self.satAssignments:
            if assignment[vindex] != val:
                continue   #this assignment can't work it doesn't make var=val
            found = True   #Otherwise it has potential. Assume found until shown otherwise
            for i, v in enumerate(self.scope()):
                if i != vindex and not v.inCurDomain(assignment[i]):
                    found = False  #Bummer...this assignment didn't work it assigns
                    break          #a value to v that is not in v's curDomain
                                   #note we skip checking if val in in var's curDomain
            if found:     #if found still true the assigment worked. We can stop
                break
        return found     #either way found has the right truth value


class QueensConstraint(Constraint):
    '''Queens constraint between queen in row i and row j'''
    def __init__(self, name, qi, qj, i, j):
        scope = [qi, qj]
        Constraint.__init__(self,name, scope)
        self._name = "QueenCnstr_" + name
        self.i = i
        self.j = j

    def check(self):
        qi = self.scope()[0]
        qj = self.scope()[1]
        if not qi.isAssigned() or not qj.isAssigned():
            return True
        return self.queensCheck(qi.getValue(),qj.getValue())

    def queensCheck(self, vali, valj):
        diag = abs(vali - valj) == abs(self.i - self.j)
        return not diag and vali != valj
    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint'''
        #hasSupport for this constraint is easier as we only have one
        #other variable in the constraint.
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in
        otherVar = self.scope()[0]
        if otherVar == var:
            otherVar = self.scope()[1]
        for otherVal in otherVar.curDomain():
            if self.queensCheck(val, otherVal):
                return True
        return False

class QueensTableConstraint(TableConstraint):
    '''Queens constraint between queen in row i and row j, but
       using a table constraint instead. That is, you
       have to create and add the satisfying tuples.

       Since we inherit from TableConstraint, we can
       call TableConstraint.__init__(self,...)
       to set up the constraint.

       Then we get hasSupport and check automatically from
       TableConstraint
    '''
    #your implementation for Question 1 goes
    #inside of this class body. You must not change
    #the existing function signatures.
    def __init__(self, name, qi, qj, i, j):
        self._name = "Queen_" + name
        # util.raiseNotDefined()
        valid_assignments = []

        for q_i in qi.domain():
            for q_j in qj.domain():
                if q_i != q_j and abs(q_i - q_j) != abs(i-j):
                    valid_assignments.append([q_i,q_j]) 


        TableConstraint.__init__(self, name, [qi,qj], valid_assignments)

        

class NeqConstraint(Constraint):
    '''Neq constraint between two variables'''
    def __init__(self, name, scope):
        if len(scope) != 2:
            print("Error Neq Constraints are only between two variables")
        Constraint.__init__(self,name, scope)
        self._name = "NeqCnstr_" + name

    def check(self):
        v0 = self.scope()[0]
        v1 = self.scope()[1]
        if not v0.isAssigned() or not v1.isAssigned():
            return True
        return v0.getValue() != v1.getValue()

    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint'''
        #hasSupport for this constraint is easier as we only have one
        #other variable in the constraint.
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in
        otherVar = self.scope()[0]
        if otherVar == var:
            otherVar = self.scope()[1]
        for otherVal in otherVar.curDomain():
            if val != otherVal:
                return True
        return False

class AllDiffConstraint(Constraint):
    '''All diff constraint between a set of variables
       If you are curious as to how to more efficiently perform GAC on
       an AllDiff see
       http://www.constraint-programming.com/people/regin/papers/alldiff.pdf'''
    def __init__(self, name, scope):
        Constraint.__init__(self,name, scope)
        self._name = "AllDiff_" + name

    def check(self):
        assignments = []
        for v in self.scope():
            if v.isAssigned():
                assignments.append(v.getValue())
            else:
                return True
        return len(set(assignments)) == len(assignments)

    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint'''
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in

        #since the contraint has many variables use the helper function 'findvals'
        #for that we need two test functions
        #1. for testing complete assignments to the constraint's scope
        #   return True if and only if the complete assignment satisfies the constraint
        #2. for testing partial assignments to see if they could possibly work.
        #   return False if the partial assignment cannot be extended to a satisfying complete
        #   assignment
        #
        #Function #2 is only needed for efficiency (sometimes don't have one)
        #  if it isn't supplied findvals will use a function that never returns False
        #
        #For alldiff, we do have both functions! And they are the same!
        #We just check if the assignments are all to different values. If not return False
        def valsNotEqual(l):
            '''tests a list of assignments which are pairs (var,val)
               to see if they can satisfy the all diff'''
            vals = [val for (var, val) in l]
            return len(set(vals)) == len(vals)
        varsToAssign = self.scope()
        varsToAssign.remove(var)
        x = findvals(varsToAssign, [(var, val)], valsNotEqual, valsNotEqual)
        return x
    
class FlightFollowsConstraint(Constraint):
    def __init__(self, name, scope, can_follow):
        Constraint.__init__(self,name, scope)
        self._name = "FlightFollows_" + name
        self.can_follow = can_follow
    #scope: [Plane1_1, Plane1_2, Plane1_3, Plane_1,4, Plane1_5]

    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint'''

        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in

        def valsFollows(l):
            plane_to_flight = {}
            # for i in range(len(l)):
            #     plane_to_flight[l[i][0].name()] = l[i][1]

            # flight_list = list(plane_to_flight.keys())
            # flight_list.sort() #-->[Plane 0,1, Plane 0,2]
            # for i in range(1,len(flight_list)):
            #     v1 = flight_list[i-1]
            #     v2 = flight_list[i]
            #     if int(v1[8]) == int(v2[8])-1: #consecutive
            #         flight1 = plane_to_flight[v1]
            #         flight2 = plane_to_flight[v2]
            #         if flight1 == "NF" and flight2 != "NF":
            #             return False
            #         if flight2!= "NF":
            #             if flight2 not in self.can_follow or flight1 not in self.can_follow[flight2]:
            #                 return False
            # return True
            plane_to_flight = [None] * len(self.scope())
            for i in range(len(l)):
                idx = int(l[i][0].name()[8])
                plane_to_flight[idx] = l[i][1]
            # flight_list = list(plane_to_flight.keys())
            # flight_list.sort() #-->[Plane 0,1, Plane 0,2]
            no_flight = False
            for i in range(1,len(plane_to_flight)):
                v1 = plane_to_flight[i-1]
                v2 = plane_to_flight[i]
                if v1 == "NF":
                    no_flight = True
                # none, NF, none, Flight1
                if no_flight and v2 is not None:
                    if v2 != "NF":
                        return False

                if v1 is None or v2 is None:
                    return False
                if v1 == "NF" and v2 != "NF":
                    return False
                if v2!= "NF":
                    if v2 not in self.can_follow or v1 not in self.can_follow[v2]:
                        return False
            return True
       
        def valsFollows2(l):
            plane_to_flight = [None] * len(self.scope())
            #TODO: if we see an earlier flight with no flight and a later flight has a flight, return False
            for i in range(len(l)):
                idx = int(l[i][0].name()[8])
                plane_to_flight[idx] = l[i][1]
            
            # flight_list = list(plane_to_flight.keys())
            # flight_list.sort() #-->[Plane 0,1, Plane 0,2]
            no_flight = False
            for i in range(1,len(plane_to_flight)):
                v1 = plane_to_flight[i-1]
                v2 = plane_to_flight[i]
                if v1 == "NF":
                    no_flight = True
                # none, NF, none, Flight1
                if no_flight and v2 is not None:
                    if v2 != "NF":
                        return False
      
                if v1 is not None and v2 is not None:
                    if v1 == "NF" and v2 != "NF":
                        return False
                    if v2!= "NF":
                        if v2 not in self.can_follow or v1 not in self.can_follow[v2]:
                            return False
            return True

        varsToAssign = self.scope()
        varsToAssign.remove(var)
        x = findvals(varsToAssign, [(var, val)], valsFollows, valsFollows2)
        return x  

class AllFlightsConstraint(Constraint):
    def __init__(self, name, scope, flights):
        Constraint.__init__(self,name, scope)
        self._name = "AllFlights_" + name
        self.flights = flights

    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint'''
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in
    
        def valsComplete(l):
            vals = [val for (var, val) in l]
            assignments = {}
            for val in vals:
                if val != "NF":
                    if val not in self.flights:
                        return False
                    assignments[val] = assignments.get(val,0) + 1
                    if assignments[val] != 1:
                        return False
            return len(assignments.keys()) == len(self.flights)
            # vals = collections.Counter(vals)
            # del vals["NF"]
            # return vals == self.flights

            
        def valsPartial(l):
            # pass
            vals = [val for (var, val) in l]
            assignments = {}
            for val in vals:
                if val != "NF":
                    if val not in self.flights:
                        return False
                    assignments[val] = assignments.get(val,0) + 1
                    if assignments[val] > 1:
                        return False
            return len(assignments.keys()) <= len(self.flights)

            # vals = collections.Counter(vals)
            # vals = vals - self.flights
            # del vals["NF"]
            # return len(vals) == 0

        varsToAssign = self.scope()
        varsToAssign.remove(var)
        x = findvals(varsToAssign, [(var, val)], valsComplete, valsPartial)
        return x 

# class NotEnoughtMaintConstraint(Constraint):
#     def __init__(self, name, scope, maint_flights, min_maint_freq):
#         Constraint.__init__(self,name, scope)
#         self._name = "NotEnoughMaint_" + name
#         self.maint_flights = maint_flights
#         self.min_maint_freq = min_maint_freq

#     def hasSupport(self, var, val):
#         '''check if var=val has an extension to an assignment of the
#            other variable in the constraint that satisfies the constraint'''
#         # if var not in self.scope():
#         #     return True   #var=val has support on any constraint it does not participate in
    
#         # def valsComplete(l):
#         #     # vals = [val for (var, val) in l]
#         #     return len(self.maint_flights) >= self.min_maint_freq 
        
#         # varsToAssign = self.scope()
#         # varsToAssign.remove(var)
#         # x = findvals(varsToAssign, [(var, val)], valsComplete, valsComplete)
#         # return x 
#         return len(self.maint_flights) >= self.min_maint_freq 

def findvals(remainingVars, assignment, finalTestfn, partialTestfn=lambda x: True):
    '''Helper function for finding an assignment to the variables of a constraint
       that together with var=val satisfy the constraint. That is, this
       function looks for a supporing tuple.

       findvals uses recursion to build up a complete assignment, one value
       from every variable's current domain, along with var=val.

       It tries all ways of constructing such an assignment (using
       a recursive depth-first search).

       If partialTestfn is supplied, it will use this function to test
       all partial assignments---if the function returns False
       it will terminate trying to grow that assignment.

       It will test all full assignments to "allVars" using finalTestfn
       returning once it finds a full assignment that passes this test.

       returns True if it finds a suitable full assignment, False if none
       exist. (yes we are using an algorithm that is exactly like backtracking!)'''

    # print("==>findvars([",)
    # for v in remainingVars: print(v.name(), " ", end='')
    # print("], [",)
    # for x,y in assignment: print("({}={}) ".format(x.name(),y), end='')
    # print("")

    #sort the variables call the internal version with the variables sorted
    remainingVars.sort(reverse=True, key=lambda v: v.curDomainSize())
    return findvals_(remainingVars, assignment, finalTestfn, partialTestfn)

def findvals_(remainingVars, assignment, finalTestfn, partialTestfn):
    '''findvals_ internal function with remainingVars sorted by the size of
       their current domain'''
    if len(remainingVars) == 0:
        return finalTestfn(assignment)
    var = remainingVars.pop()
    for val in var.curDomain():
        assignment.append((var, val))
        if partialTestfn(assignment):
            if findvals_(remainingVars, assignment, finalTestfn, partialTestfn):
                return True
        assignment.pop()   #(var,val) didn't work since we didn't do the return
    remainingVars.append(var)
    return False


class NValuesConstraint(Constraint):
    '''NValues constraint over a set of variables.  Among the variables in
       the constraint's scope the number that have been assigned
       values in the set 'required_values' is in the range
       [lower_bound, upper_bound] (lower_bound <= #of variables
       assigned 'required_value' <= upper_bound)

       For example, if we have 4 variables V1, V2, V3, V4, each with
       domain [1, 2, 3, 4], then the call
       NValuesConstraint('test_nvalues', [V1, V2, V3, V4], [3,2], 2,
       3) will only be satisfied by assignments such that at least 2
       the V1, V2, V3, V4 are assigned the value 3 or 2, and at most 3
       of them have been assigned the value 3 or 2.

    '''

    def __init__(self, name, scope, required_values, lower_bound, upper_bound):
        Constraint.__init__(self,name, scope)
        self._name = "NValues_" + name
        self._required = required_values
        self._lb = lower_bound
        self._ub = upper_bound

    def check(self):
        # util.raiseNotDefined()
        num_req = 0
        for var in self.scope():
            if not var.isAssigned():
                return True
            if var.getValue() == self._required:
                num_req+=1
        return self._lb <= num_req <= self._ub


    def hasSupport(self, var, val):
        '''check if var=val has an extension to an assignment of the
           other variable in the constraint that satisfies the constraint

           HINT: check the implementation of AllDiffConstraint.hasSupport
                 a similar approach is applicable here (but of course
                 there are other ways as well)
        '''
        if var not in self.scope():
            return True   #var=val has support on any constraint it does not participate in

        def valsComplete(l):
            vals = [val for (var, val) in l]
            num_req = 0

            for val in vals:
                if val in self._required:
                    num_req+=1
            return self._lb <= num_req <= self._ub
        
        def valsPartial(l):
            vals = [val for (var, val) in l]
            num_req = 0

            for val in vals:
                if val in self._required:
                    num_req+=1
            return num_req <= self._ub


        varsToAssign = self.scope()
        varsToAssign.remove(var)
        x = findvals(varsToAssign, [(var, val)], valsComplete, valsPartial)
        return x
