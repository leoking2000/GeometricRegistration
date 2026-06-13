#include <geo/GeometricRegistration.h>
#include <iostream>

struct Args
{
    std::string source;
    std::string target;
    std::string output;

    std::string method = "ESA";
    geo::u32 maxIter = 100;

    bool verbose = false;
};

static Args ParseArgs(int argc, char** argv)
{
    Args args;

    for (int i = 1; i < argc; i++)
    {
        std::string key = argv[i];

        auto next = [&](std::string& out)
            {
                if (i + 1 < argc)
                    out = argv[++i];
            };

        auto nextInt = [&](geo::u32& out)
            {
                if (i + 1 < argc)
                    out = std::stoi(argv[++i]);
            };

        auto nextFloat = [&](geo::f32& out)
            {
                if (i + 1 < argc)
                    out = std::stof(argv[++i]);
            };

        if (key == "-source")
        {
            next(args.source);
        }
        else if (key == "-target")
        {
            next(args.target);
        }
        else if (key == "-o" || key == "-output")
        {
            next(args.output);
        }
        else if (key == "-method")
        {
            next(args.method);
        }
        else if (key == "-maxIter")
        {
            nextInt(args.maxIter);
        }
        else if (key == "-v" || key == "-verbose")
        {
            args.verbose = true;
        }
    }

    return args;
}

static void PrintArgs(const Args& a)
{
    std::cout << "\n========== GeoAlign Args ==========\n";
    std::cout << "Source:   " << a.source << "\n";
    std::cout << "Target:   " << a.target << "\n";
    std::cout << "Output:   " << a.output << "\n";
    std::cout << "Method:   " << a.method << "\n";
    std::cout << "MaxIter:  " << a.maxIter << "\n";
    std::cout << "Verbose:  " << (a.verbose ? "true" : "false") << "\n";
    std::cout << "===================================\n\n";
}

int main(int argc, char** argv)
{
    Args args = ParseArgs(argc, argv);

    PrintArgs(args);

    return 0;
}

