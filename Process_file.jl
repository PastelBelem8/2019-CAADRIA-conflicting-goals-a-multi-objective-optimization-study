using DelimitedFiles

base_dir = "final/Analises"


process(algorithm, base_dir=base_dir) = let
    println("Received ARGS: $(algorithm...)")
    runs = [1, 2, 3]

    get_filename(a, r) = "$(base_dir)/$(a)_results_0$(r).csv"

    filenames = map(c -> begin print(c); get_filename(c[1], c[2]) end, Iterators.product(algorithm, runs))
    # println(filenames)
    filenames = filter(isfile, filenames)
    println("About to join the following files:\n", join(filenames, '\n'))

    read(;filename, header=true) = open(filename, "r") do f
                                       header ? readline(f) : nothing
                                       readdlm(f, ',')
                                   end

    read_cols(; filename, cols, header=true) = read(filelanme=filename, header=header)[:, cols]

    output_folder = "final/outputs/algorithms"
    output_file = "$(output_folder)/all_$(algorithm...)_solutions.csv"

    println("Concatenating all files in a single one called `$(output_file)`...")
    all_data = vcat(map(f -> read(filename=f, header=true), filenames)...)
    # all_data = vcat(map(f -> read(filename=f, header=true)[101:end,:], filenames)...)

    open(output_file, "w") do io
               writedlm(io, all_data, ',')
    end;
end

algorithms = ["GPR_NSGAII", "GPR_Random", "GPR_SMPSO", "MLP_NSGAII", "MLP_Random", "MLP_SMPSO", "RF_NSGAII", "RF_Random", "RF_SMPSO"]
algorithms
# algorithms = ["OMOPSO", "SMPSO", "MOEAD", "SPEA2", "EpsMOEA", "PESA2", "PAES", "NSGAII", "GDE3", "CMAES"]


for alg in algorithms
    process([alg])
end
