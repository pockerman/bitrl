# Evolution strategies: a search for the parameters of a policy that needs only
# the returns of episodes, no gradients. The episodes of a generation do not
# depend on each other, so they run on all the threads, or on several ranks
# that exchange only the returns.
bitrl_add_example(es_cart_pole_swing_up SMOKE_TEST SMOKE_ARGS --generations 60)
bitrl_add_example(es_distributed SMOKE_TEST SMOKE_ARGS --generations 20)
