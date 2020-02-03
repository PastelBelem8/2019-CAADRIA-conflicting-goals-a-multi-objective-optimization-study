using Khepri

fixed_xyz_truss_node_family =
  truss_node_family_element(default_truss_node_family(),
                            support=create_node_support("SupportA", ux=true, uy=true, uz=true))

fixed_z_truss_node_family =
  truss_node_family_element(default_truss_node_family(),
                            support=create_node_support("SupportB", ux=false, uy=false, uz=true))

default_truss_bar_family(
  truss_bar_family_element(
    default_truss_bar_family(),
    material=[
      "ElasticIsotropic",   # name
      Khepri.I_MT_STEEL,    # Type
      "Steel",              # Name
      "I'm really steel",   # Nuance
      210000000000.0,       # E
      0.3,                  # NU
      81000000000.0,        # Kirchoff
      77010.0,              # RO
      1.2e-05,              # LX
      0.04,                 # DumpCoef
      235000000.0,          # RE
      360000000.0],         # RT
    section=[
      "Tube",               #name
      "ElasticIsotropic",   #material_name
      false,                #iswood
      [(true,               #solid?
        0.1,                #diameter
        0.01)]]))           #thickness

no_trelica(p) = truss_node(p)

fixed_xyz_no_trelica(p) = truss_node(p, fixed_xyz_truss_node_family)

fixed_z_no_trelica(p) = truss_node(p, fixed_z_truss_node_family)

barra_trelica(p0, p1) = truss_bar(p0, p1)

#;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
#TRELIÇA

nos_trelica(ps) =
  for p in ps
    no_trelica(p)
  end

fixed_nos_trelica(ps) =
  begin
    fixed_xyz_no_trelica(ps[1])
    nos_trelica(ps[2:end-1])
    fixed_xyz_no_trelica(ps[end])
  end

barras_trelica(ps, qs) =
  for (p, q) in zip(ps, qs)
    barra_trelica(p, q)
  end

trelica_espacial(curvas, f, isfirst=true) =
  let (as, bs, cs) = (curvas[1], curvas[2], curvas[3])
    isfirst ? fixed_nos_trelica(as) : nos_trelica(as)
    nos_trelica(bs)
    curvas[4:end] == [] ?
      (fixed_nos_trelica(cs);
       barras_trelica(cs[2:end], cs)) :
      (trelica_espacial(curvas[3:end], f, false);
       barras_trelica(bs, curvas[4]))
    barras_trelica(as, cs)
    barras_trelica(bs, as)
    barras_trelica(bs, cs)
    barras_trelica(bs, as[2:end])
    barras_trelica(bs, cs[2:end])
    barras_trelica(as[2:end], as)
    barras_trelica(bs[2:end], bs)
  end


attractors = Parameter([xyz(5, 5, 5)])
#TRELIÇA
affect_radius(r, p) =
  r*(1+0.5*1.4^(-min(map(attractor -> distance(p, attractor), attractors())...)))

pontos_arco(p, r, fi, psi0, psi1, dpsi) =
  psi0 > psi1 ?
    [] :
    vcat([p+vsph(affect_radius(r, p+vsph(r, fi, psi0)), fi, psi0)],
         pontos_arco(p, r, fi, psi0+dpsi, psi1, dpsi))

coordenadas_trelica_ondulada(p, rac, rb, l, fi, psi0, psi1, dpsi, alfa0, alfa1, d_alfa, d_r) =
  alfa0 >= alfa1 ?
    [pontos_arco(p+vpol(l/2.0, fi-pi/2), rac+d_r*sin(alfa0), fi, psi0, psi1, dpsi)] :
    vcat([pontos_arco(p+vpol(l/2.0, fi-pi/2), rac+d_r*sin(alfa0), fi, psi0, psi1, dpsi)],
         vcat([pontos_arco(p, rb+d_r*sin(alfa0), fi, psi0+dpsi/2, psi1-dpsi/2, dpsi)],
              coordenadas_trelica_ondulada(p+vpol(l, fi+pi/2), rac, rb, l, fi, psi0, psi1, dpsi, alfa0+d_alfa, alfa1, d_alfa, d_r)))

trelica_ondulada(p, rac, rb, l, n, fi, psi0, psi1, alfa0, alfa1, d_alfa, d_r) =
  trelica_espacial(coordenadas_trelica_ondulada(p, rac, rb, l, fi, psi0, psi1, (psi1-psi0)/n, alfa0, alfa1, d_alfa, d_r),
                   panel)

#
using LinearAlgebra
using Combinatorics

backend(robot)
project_kind(Khepri.I_PT_SHELL)

spiked_truss_displacement(α1, y1, α2, y2, α3, y3, α4, y4, α5, y5) =
  let limit = 10pi
      delta = 1.0
      attrs = [ sph(10, 0, α1)+vy(y1),
                sph(10, 0, α2)+vy(y2),
                sph(10, 0, α3)+vy(y3),
                sph(10, 0, α4)+vy(y4),
                sph(10, 0, α5)+vy(y5)]
    with(attractors, attrs) do
      delete_all_shapes()
      new_robot_analysis(
        ()-> trelica_ondulada(xyz(0, 0, 0),10,9,delta,10,0,-pi/2,pi/2,0,limit,delta,0.0),
        vxyz(-50000.0, 0.0, 0.0)) do results
          let displs = displacements(nodes(results))
              node_displ = node ->
                norm(node_displacement_vector(displs, node.id, Khepri.I_LRT_NODE_DISPLACEMENT))
              disps = maximum([node_displ(node) for node in values(added_nodes())])
              # style = maximum(distance(p0, p1) for (p0, p1) in combinations(attractors(), 2))
            with(current_backend, autocad) do
              delete_all_shapes()
              show_truss_deformation(results)
            end
            disps# , style
        end
      end
    end
  end

spiked_truss_style(α1, y1, α2, y2, α3, y3, α4, y4, α5, y5) = begin
  with(attractors, [sph(10, 0, α1)+vy(y1),
                    sph(10, 0, α2)+vy(y2),
                    sph(10, 0, α3)+vy(y3),
                    sph(10, 0, α4)+vy(y4),
                    sph(10, 0, α5)+vy(y5)]) do
    sum(distance(p0, p1) for (p0, p1) in combinations(attractors(), 2))
  end
end


backend(autocad)

spiked_truss(α1, y1, α2, y2, α3, y3, α4, y4, α5, y5) =
  let limit = 10pi
      delta = 1.0
      attrs = [ sph(10, 0, α1)+vy(y1),
                sph(10, 0, α2)+vy(y2),
                sph(10, 0, α3)+vy(y3),
                sph(10, 0, α4)+vy(y4),
                sph(10, 0, α5)+vy(y5)]
      with(attractors, attrs) do
        delete_all_shapes()
        trelica_ondulada(xyz(0, 0, 0),10,9,delta,10,0,-pi/2,pi/2,0,limit,delta,0.0)
    end
  end


produce_truss(i, A, B) = begin
    println("Vars: $(A[:, i])")
    spiked_truss(A[:, i]...)
    println("Objs: $(B[:, i])")
  end

spiked_truss(pi/4, 5, -pi/3, 20, pi/10, 25, -pi/5, 5, pi/2, 15)

# using DelimitedFiles

# filenames = [
#             "NSGAII_results_01.csv",
#             "NSGAII_results_02.csv",
#             "NSGAII_results_03.csv",
#             "SPEA2_results_01.csv",
#             "SPEA2_results_02.csv",
#             "SPEA2_results_03.csv"
#             ]
#
# get_Data(f, header=true) = open(f, "r") do io
#             header ? readline(io) : nothing
#             readdlm(io, ',', Float64, '\n')
#           end
# #
# # data = vcat(map(get_Data, filenames)...)
# dump_data(f, d) = open(f, "w") do io
#             writedlm(io, d, ',')
#           end
#
# # dump_data("NSGA_SPEA_all.csv", data)
#
# data = get_Data("All_non_dominated.csv", false)'
# vars = collect(1:6)
# objs = collect(7:8)
#
#
# X = data[vars, :]
# Y = data[objs, :]
#
# produce_truss(8, X, Y)

# produce_truss(i) = begin
#   println("Vars: $(X[:,i])")
#   spiked_truss(X[:, i]...)
#   println("Objs: Displacement: $(ys[1,i]), Sum distance: $(ys[2,i])")
# end
#
# produce_truss(3)
#=
println(spiked_truss_displacement(pi/4, 5, -pi/3, 20, pi/10, 25))
println(spiked_truss_style(pi/4, 5, -pi/3, 20, pi/10, 25))
(1.2648155430158327, 21.834921777293)

println(spiked_truss_displacement(pi/2, 1, pi/3, 3, pi/10, 5))
println(spiked_truss_style(pi/2, 1, pi/3, 3, pi/10, 5))
(1.3258518106063275, 12.417592404528765)
=#

# Minimizar displacements
# Maximizar style
#=

include MscThesis
# Step 1. Define the Model
vars = [RealVariable(-π, π), RealVariable(0, 4π),
        RealVariable(-π, π), RealVariable(0, 4π),
        RealVariable(-π, π), RealVariable(0, 4π)]
objs = [Objective(x -> spiked_truss_displacement(x[1], x[2], x[3], x[4], x[5], x[6]), 1, :MIN),
        Objective(x -> spiked_truss_style(x[1], x[2], x[3], x[4], x[5], x[6]), :MAX)]

model = Model(vars, objs)

# Step 2. Define the Solver

a_type = NSGAII;
a_params = Dict(:population_size => 5);
solver = PlatypusSolver(a_type, max_eval=5, algorithm_params=a_params)

# Step 3. Solve it!
sols = solve(solver, model)

using Dates
using Main.MscThesis
using Main.MscThesis.Metamodels
using Main.MscThesis.Platypus
using Main.MscThesis.Sampling

vars = [RealVariable(-π, π), RealVariable(0, 4π),
        RealVariable(-π, π), RealVariable(0, 4π),
        RealVariable(-π, π), RealVariable(0, 4π)]
nvars = length(vars)
# objs = [Objective(x -> spiked_truss_displacement(x[1], x[2], x[3], x[4], x[5], x[6]), 1, :MIN),
#         Objective(x -> spiked_truss_style(x[1], x[2], x[3], x[4], x[5], x[6]), :MAX)]
objs1 = (Objective(x -> spiked_truss_displacement(x[1], x[2], x[3], x[4], x[5], x[6]), 1, :MIN),
        Objective(x -> -spiked_truss_style(x[1], x[2], x[3], x[4], x[5], x[6]), :MIN))
nobjs = length(objs1)

model = LinearRegression(multi_output=true)
surrogate = Surrogate(model, objectives=objs1)

meta_problem = MetaModel(vars, [surrogate])

# Sampling Params
sampling_params = Dict{Symbol, Any}(
  :sampling_function => Sampling.latinHypercube,
  :filename => "$(Dates.format(Dates.now(), "yyyymmddHHMMSS")).csv",
  :nsamples => 2,
  :clip => true,
  :nbins => 2)

# Step 2. Define the Solver
a_type = OMOPSO
a_params = Dict(:epsilons=>[0.1, 0.5],
                :swarm_size => 10,
                :leader_size => 10,
                :max_iterations => 50,
                :mutation_probability => 0.3,
                :mutation_perturbation => 0.5)
solver = Main.MscThesis.PlatypusSolver(a_type, max_eval=250, algorithm_params=a_params)
meta_solver = MetaSolver(solver, nvars=nvars, nobjs=nobjs, max_eval=130, sampling_params=sampling_params)

for i in 1:3
    @info "============================ Starting run $i for algorithm LR with $(string(a_type)) =================================="
    Main.MscThesis.csv_file("$(string(a_type))_LinearRegression_results_0$(i).csv")
    Main.MscThesis.csv_write(["Total time (s)", "v1", "v2", "v3", "v4", "v5", "v6", "O1", "O2"], "a")
    Main.MscThesis.solve(meta_solver, meta_problem)
end

=#
