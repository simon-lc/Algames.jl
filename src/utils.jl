# Run tests locally
using Pkg
Pkg.activate("/home/simon/.julia/dev/Algames")
# Pkg.test("Algames")


Pkg.activate("/home/simon/.julia/dev/Algames/test")


function scn(a::Number; digits::Int=1)
    # a = m x 10^e
    if a == 0
        e = 0
        m = 0.0
    else
        e = Int(floor(log(abs(a))/log(10)))
        m = a*exp(-e*log(10))
    end
    m = round(m, digits=digits)
    if digits == 0
        m = Int(floor(m))
    end
    sgn = a >= 0 ? " " : ""
    sgne = e >= 0 ? "+" : ""
    return "$sgn$(m)e$sgne$e"
end
