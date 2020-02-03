using DelimitedFiles

base_dir = "final/Analises"
# algorithms = ["OMOPSO", "SMPSO", "MOEAD", "SPEA2", "EpsMOEA", "PESA2", "PAES", "NSGAII"]
algorithms = ["OMOPSO", "SMPSO", "MOEAD", "SPEA2", "EpsMOEA", "PESA2", "PAES", "NSGAII", "GDE3", "CMAES"]
algorithms = vcat(algorithms, ["GPR_NSGAII", "GPR_LHS", "GPR_SMPSO", "MLP_NSGAII", "MLP_Random", "MLP_SMPSO", "RF_NSGAII", "RF_Random", "RF_SMPSO"])
algorithms
runs = [1, 2, 3]

get_filename(a, r) = "$(base_dir)/$(a)_results_0$(r).csv"

filenames = map(c -> get_filename(c[1], c[2]), Iterators.product(algorithms, runs))
filenames = filter(isfile, filenames)
println("About to join the following files:\n", join(filenames, '\n'))

read(;filename, header=true) = open(filename, "r") do f
                                   header ? readline(f) : nothing
                                   readdlm(f, ',')
                               end

read_cols(; filename, cols, header=true) = read(filelanme=filename, header=header)[:, cols]

println("Reading file: $(filenames[1])")
f1_data = read(filename=filenames[1])
println("Size of collected data: $(size(f1_data))")
println("Type of collected data: $(typeof(f1_data))")

output_folder = "final/outputs"
output_file = "$(output_folder)/all_solutions.csv"
println("Concatenating all files in a single one called `$(output_file)`...")
all_data = vcat(map(f -> read(filename=f, header=true), filenames)...)

open(output_file, "w") do io
           writedlm(io, all_data, ',')
end;
