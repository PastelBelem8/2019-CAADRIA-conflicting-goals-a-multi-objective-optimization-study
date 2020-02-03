 This folder contains the three runs for 15 iterations with 15-sized populations. 

The algorithms considered for CAADRIA 2019 were:
 - EpsMOEA (Epsilon Dominance based MOEA)
 - MOEAD   (MOEA with a Decomposition factor) 
 - NSGAII  (Non-Dominated Sorting Genetic Algorithm II)
 - OMOPSO  (Multi-Objective Particle Swarm Optimizer) 
 - PAES    (Pareto Archived Evolution Strategy)
 - PESA2   (Pareto Envelope-based Selection Algorithm 2)
 - SMPSO   (Speed-constrained Multi-Objective Particle Swarm Optimization)
 - SPEA2   (Strength Pareto Evolutionary Algorithm)

We used the Python's framework Platypus implementation of these algorithms and implemented a Julia binding to invoke them directly from Khepri.

# We then proceeded to test two more algorithms
  - CMAES	(Co-variance Matrix Adaptive Evolution Strategy)
  - GDE3 	(Differential Evolution 3)