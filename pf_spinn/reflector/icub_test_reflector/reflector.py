import spynnaker.pyNN as p

p.setup(1.0, n_chips_required=2)
print "starting work"
# shouldnt need this, but we do. oops.
p.Population(1, p.IF_curr_exp, {}, label="Stupid requirement ;-)")
p.run() # 100000 change the time to none if you want to run it forever (then p.end() not needed)
print "woop!"
#p.end()
