import env
import opponents as op
import sample_probs as sp


# Select Between Different Mazes:
#
# sp.rect20a
# sp.rect20b
# sp.rect20c
# sp.rect20d
# sp.rect50
# sp.rect100
# sp.wall16a
# sp.wall16b
# sp.lhook32
# sp.rhook32a
# sp.rhook32b
# sp.spiral32
# sp.pdes30
# sp.pdes30b
# sp.rectwall32
# sp.rectwall32a
# sp.walls32
# sp.twisty1

# Select Between Two Opponents:
#
# op.opponent0 (Stupid)
# op.opponent1 (Smart)

env.main(sp.spiral32, max_search_time=5, max_init_time=5, opponent=op.opponent0)
