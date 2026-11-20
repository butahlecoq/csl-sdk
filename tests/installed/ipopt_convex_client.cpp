#include <csl/adapters/convex/ipopt.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
using namespace csl;
using namespace csl::optimization;
using namespace csl::adapters::convex;

void require(bool condition, const char *label) {
    if (!condition)
        throw std::runtime_error(label);
}

ProgramOutcome solve(const ConvexProgram &program, const ExecutionContext &context,
                     IpoptConvexOptions options = {}) {
    auto provider =
        std::get<registry::Registration<registry::ConvexPort>>(make_ipopt_registration(options));
    require(provider.identity.id.value == "csl.convex.ipopt", "provider identity");
    return provider.factory(program)->solve(program, context);
}

ConvexProgram program(double rhs, Interval limit) {
    return std::get<ConvexProgram>(
        make_convex_program({{{"installed-ipopt"}, {"source-qp"}},
                             QuadraticTerminal{{2, 2, {2, 0, 0, 2}}, {-6, -2}, 10},
                             {1, 2, {0, 2}, {0, 1}, {1, std::ldexp(1.0, -80)}},
                             {limit, {0, 5}},
                             {{rhs, rhs}},
                             {}}));
}
}

int main() {
    const ExecutionContext context{
        std::make_shared<const ResourcePolicy>(ResourcePolicy{1, 1, 1, {}}),
        {},
        {},
        {1e-8, 1e-8},
        "ipopt-installed"};
    const auto source = program(2, {0, 5});
    const auto hinted = std::get<ConvexProgram>(with_primal_start(source, Vector{2, 0}));
    for (const auto *p : {&source, &hinted}) {
        const auto result = std::get<ProgramResult>(solve(*p, context));
        require(result.termination == Termination::completed, "completed solve");
        require(result.outcome.value == MathematicalOutcome::optimal, "numerical optimum");
        require(result.outcome.subject == p->identity(), "original program identity");
        require(result.primal && result.residuals.primal && result.residuals.dual,
                "original KKT evidence");
        require(check_program_result(result, *p, Assurance::numerical).empty(), "evidence binding");
        require(std::abs(result.primal->at(0) - 2) < 1e-8 &&
                    std::abs(result.primal->at(1) - 1) < 1e-8,
                "original primal coordinates");
        if (p == &hinted)
            require(result.raw_diagnostics.find("primal_start=submitted") != std::string::npos,
                    "hint submitted");
    }

    const auto infeasible = std::get<ProgramResult>(solve(program(2, {0, 1}), context));
    require(infeasible.outcome.value == MathematicalOutcome::undetermined,
            "native infeasibility is unresolved");
    require(infeasible.raw_diagnostics.find("ipopt_status=2") != std::string::npos,
            "native infeasibility status");

    const auto limited = std::get<ProgramResult>(solve(source, context, {{0}}));
    require(limited.termination == Termination::resource_limit, "native iteration budget");
    std::stop_source token;
    token.request_stop();
    auto stopped = context;
    stopped.cancellation = token.get_token();
    require(std::get<ProgramResult>(solve(source, stopped)).termination == Termination::cancelled,
            "pre-cancelled solve");
    stopped = context;
    stopped.deadline = std::chrono::steady_clock::now();
    require(std::get<ProgramResult>(solve(source, stopped)).termination ==
                Termination::deadline_expired,
            "expired deadline");

    require(std::holds_alternative<Unsupported>(solve(program(2, {0, 1e19}), context)),
            "finite bound rejected at infinity convention");
    require(std::holds_alternative<Diagnostics>(
                make_ipopt_registration({{std::numeric_limits<std::size_t>::max()}})),
            "iteration count fits native integer");
}
