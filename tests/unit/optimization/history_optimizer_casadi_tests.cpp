#include <doctest/doctest.h>

#include <csl/adapters/nlp/casadi_history_optimizer.hpp>
#include <csl/sets/data.hpp>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace csl;

namespace {
template <class T, class... E> T take(std::variant<T, E...> result) {
    REQUIRE(std::holds_alternative<T>(result));
    return std::get<T>(std::move(result));
}
optimization::HistoryOptimizationRequest integrator_request(double running_weight) {
    constexpr std::size_t n = 8;
    Matrix inequalities{4, n, {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -1, 0,
                               0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,  -1}};
    Vector upper{1, 0, 1, 0};
    Matrix equalities{6, n, {1,  0, 0,  0, 0, 0, 0,  0, 0, 1,  0, 0,  0, 0, 0, 0,
                             -1, 0, 1,  0, 0, 0, -1, 0, 0, -1, 0, 1,  0, 0, 0, 0,
                             0,  0, -1, 0, 1, 0, 0,  0, 0, 0,  0, -1, 0, 1, 0, -1}};
    Vector rhs(6, 0.0);
    auto set = take(
        sets::make_h({{"two-step-integrator"}, n, {}},
                     {take(numeric::snapshot(inequalities)), take(numeric::snapshot_vector(upper)),
                      take(numeric::snapshot(equalities)), take(numeric::snapshot_vector(rhs))}));
    synthesis::JointHistoryRelation histories{{{2, 1, 0}, 2}, std::move(set)};
    TerminalObjective terminal = ResidualNormTerminal{{2, 2, {1, 0, 0, 1}}, {1, 1}, {1, 1}};
    optimization::RunningControlCost running{
        "half-l1-control",
        2,
        {{0, {optimization::EuclideanControlPenalty{{1, 1, {1}}, {0}, running_weight}}},
         {1, {optimization::EuclideanControlPenalty{{1, 1, {1}}, {0}, running_weight}}}}};
    const auto compiled = optimization::compile_history_cost(histories, terminal, running, 0);
    if (!std::holds_alternative<optimization::HistoryCostProgram>(compiled))
        throw std::runtime_error("2D integrator source cost did not compile");
    return {{ProblemId{"2d-integrator"}, ProgramId{"terminal-l2-plus-running-l1"}},
            {histories, {}},
            std::get<optimization::HistoryCostProgram>(compiled),
            optimization::SuppliedStart{{0, 0, 0.5, 0, 0.5, 0.5, 0.5, 0.5}}};
}
}

TEST_CASE("CasADi/IPOPT requires a supplied feasible full-history start") {
    auto request = integrator_request(0.5);
    request.start = optimization::ColdStart{};
    auto registration = adapters::nlp::make_casadi_history_optimizer_registration();
    auto workspace = registration.factory(request);
    const auto result =
        registry::HistoryOptimizerPort::run(*workspace, request, ExecutionContext{});
    REQUIRE(std::holds_alternative<Diagnostic>(result));
    CHECK(std::get<Diagnostic>(result).field == "history.start");
}

TEST_CASE("running cost can choose a different history from the terminal-only minimizer") {
    auto request = integrator_request(2.0);
    auto registration = adapters::nlp::make_casadi_history_optimizer_registration();
    auto workspace = registration.factory(request);
    const auto result =
        registry::HistoryOptimizerPort::run(*workspace, request, ExecutionContext{});
    if (const auto *diagnostic = std::get_if<Diagnostic>(&result)) {
        INFO(diagnostic->field);
        FAIL_CHECK(diagnostic->explanation);
    }
    REQUIRE(std::holds_alternative<optimization::HistoryOptimizationResult>(result));
    const auto &solved = std::get<optimization::HistoryOptimizationResult>(result);
    CHECK(solved.source_cost.total == doctest::Approx(std::sqrt(2.0)).epsilon(1e-6));
    CHECK(solved.history[6] == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(solved.history[7] == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("CasADi/IPOPT solves full history with typed selected-state memberships") {
    using namespace optimization;
    auto request = integrator_request(2.0);
    request.domain.state_frame = "x-y";
    auto &start = std::get<SuppliedStart>(request.start);
    const auto origin = [](HistorySetFamily family, const char *id) {
        return HistoryMembershipOrigin{{id}, {id}, family, SetRelation::inner, "x-y", {2, 3}};
    };
    SUBCASE("polytope H rows") {
        auto set = take(sets::make_h({{"selected-h"}, 2, {}},
                                     {take(numeric::snapshot(Matrix{2, 2, {1, 0, -1, 0}})),
                                      take(numeric::snapshot_vector(Vector{0.7, -0.3})),
                                      take(numeric::snapshot(Matrix{0, 2, {}})),
                                      take(numeric::snapshot_vector(Vector{}))}));
        request.domain.memberships.push_back(
            {{1}, origin(HistorySetFamily::polytope, "selected-h"), HistoryHMembership{set}});
        start.membership_witnesses.push_back(HistoryHWitness{});
    }
    SUBCASE("bounded factor zonotope") {
        request.domain.memberships.push_back(
            {{1},
             origin(HistorySetFamily::zonotope, "selected-z"),
             HistoryZonotopeMembership{{0.5, 0}, {2, 2, {0.2, 0, 0, 0}}}});
        start.membership_witnesses.push_back(HistoryZonotopeWitness{{0, 0}});
    }
    SUBCASE("axis-aligned box") {
        request.domain.memberships.push_back({{1},
                                              origin(HistorySetFamily::box, "selected-b"),
                                              HistoryAxisBoxMembership{{0.5, 0}, {0.2, 0}}});
        start.membership_witnesses.push_back(HistoryAxisBoxWitness{{0, 0}});
    }
    SUBCASE("degenerate parallelotope with full-rank frame") {
        request.domain.memberships.push_back(
            {{1},
             origin(HistorySetFamily::parallelotope, "selected-p"),
             HistoryParallelotopeMembership{{0.5, 0}, {2, 2, {1, 0, 0, 1}}, {-0.2, 0}, {0.2, 0}}});
        start.membership_witnesses.push_back(HistoryParallelotopeWitness{{0, 0}});
    }
    SUBCASE("rank-deficient factor ellipsoid") {
        request.domain.memberships.push_back(
            {{1},
             origin(HistorySetFamily::ellipsoid, "selected-e"),
             HistoryBallFactorMembership{{{0.5, 0}, {2, 2, {0.2, 0, 0, 0}}}}});
        start.membership_witnesses.push_back(HistoryBallFactorWitness{{0, 0}});
    }
    SUBCASE("bounded V simplex") {
        auto set = take(sets::make_v({{"selected-v"}, 2, {}},
                                     {take(numeric::snapshot(Matrix{2, 2, {0.3, 0, 0.7, 0}})),
                                      take(numeric::snapshot(Matrix{0, 2, {}}))}));
        request.domain.memberships.push_back({{1},
                                              origin(HistorySetFamily::polytope, "selected-v"),
                                              HistoryVSimplexMembership{set}});
        start.membership_witnesses.push_back(HistoryVSimplexWitness{{0.5, 0.5}});
    }
    for (auto &member : request.domain.memberships)
        member.origin.source = HistorySourceGeometry{member.origin.reference,
                                                     member.origin.selected_family, member.shape};
    auto plain = adapters::nlp::make_casadi_history_optimizer_registration();
    auto scaled =
        adapters::nlp::make_casadi_scaled_history_optimizer_registration({{100.0, 0.01}, {0.5}});
    CHECK(plain.identity.id != scaled.identity.id);
    for (auto &registration : {std::ref(plain), std::ref(scaled)}) {
        auto workspace = registration.get().factory(request);
        const auto outcome =
            registry::HistoryOptimizerPort::run(*workspace, request, ExecutionContext{});
        if (const auto *diagnostic = std::get_if<Diagnostic>(&outcome)) {
            INFO(diagnostic->field);
            FAIL_CHECK(diagnostic->explanation);
        }
        REQUIRE(std::holds_alternative<HistoryOptimizationResult>(outcome));
        const auto &result = std::get<HistoryOptimizationResult>(outcome);
        CHECK(result.history[2] == doctest::Approx(0.3).epsilon(1e-5));
        CHECK(result.history[6] == doctest::Approx(0.3).epsilon(1e-5));
        CHECK(result.history[7] == doctest::Approx(0.0).epsilon(1e-5));
        CHECK(result.source_cost.total ==
              doctest::Approx(std::hypot(0.7, 1.0) + 0.6).epsilon(1e-5));
        REQUIRE(result.membership_witnesses.size() == 1);
        const auto &readback = std::get<VerifiedSuppliedStart>(result.initialization);
        CHECK(readback.membership_witness_readback == start.membership_witnesses);
        CHECK(readback.primal_readback == start.history);
        const auto scale_evidence =
            result.diagnostics.find("state_scales=[100,0.01]; control_scales=[0.5]");
        CHECK((scale_evidence != std::string::npos) == (&registration.get() == &scaled));
    }
}

TEST_CASE("Scaled CasADi admits only positive finite scales for every physical coordinate") {
    using adapters::nlp::make_casadi_scaled_history_optimizer_registration;
    const auto request = integrator_request(0.5);
    const auto check = [&](Vector state, Vector control) {
        auto registration = make_casadi_scaled_history_optimizer_registration(
            {std::move(state), std::move(control)});
        return registration.capabilities(request, {});
    };
    CHECK(std::holds_alternative<registry::Offer>(check({100., 0.01}, {0.5})));
    CHECK(std::holds_alternative<Diagnostics>(check({100.}, {0.5})));
    CHECK(std::holds_alternative<Diagnostics>(check({100., 0.01}, {})));
    CHECK(std::holds_alternative<Diagnostics>(check({0., 0.01}, {0.5})));
    CHECK(std::holds_alternative<Diagnostics>(
        check({100., std::numeric_limits<double>::infinity()}, {0.5})));
}

TEST_CASE("Explicit MUMPS dependency detection retains the full original equality history") {
    using namespace optimization;
    auto request = integrator_request(2.0);
    const auto &old = request.domain.original.relation;
    const auto rep = std::get<sets::HRepresentation>(old.representation());
    Matrix eq{12, 8, {}};
    Vector values;
    for (unsigned copy = 0; copy < 2; ++copy) {
        for (std::size_t row = 0; row < rep.equalities.rows(); ++row) {
            for (std::size_t col = 0; col < rep.equalities.cols(); ++col)
                eq.values.push_back(rep.equalities(row, col));
            values.push_back(rep.equality_values[row]);
        }
    }
    request.domain.original.relation =
        take(sets::make_h({old.metadata()->id, 8, {}},
                          {rep.inequalities, rep.upper_bounds, take(numeric::snapshot(eq)),
                           take(numeric::snapshot_vector(values))}));
    const auto &start = std::get<SuppliedStart>(request.start);
    auto registration = adapters::nlp::make_casadi_mumps_history_optimizer_registration();
    CHECK(registration.identity.id.value == "csl.nlp.casadi-ipopt.mumps-history-cost");
    REQUIRE(std::holds_alternative<registry::Offer>(registration.capabilities(request, {})));
    auto worker = registration.factory(request);
    const auto outcome = registry::HistoryOptimizerPort::run(*worker, request, ExecutionContext{});
    if (const auto *error = std::get_if<Diagnostic>(&outcome))
        INFO(error->explanation);
    REQUIRE(std::holds_alternative<HistoryOptimizationResult>(outcome));
    const auto &solved = std::get<HistoryOptimizationResult>(outcome);
    CHECK(solved.source_cost.total == doctest::Approx(std::sqrt(2.0)).epsilon(1e-6));
    CHECK(solved.diagnostics.find("dependency_detector=mumps; dependency_detection_with_rhs=yes") !=
          std::string::npos);
    CHECK(std::get<VerifiedSuppliedStart>(solved.initialization).primal_readback == start.history);
}

TEST_CASE("CasADi returns a witness for a lifted H view of its original history") {
    using namespace optimization;
    auto request = integrator_request(2.0);
    request.domain.state_frame = "x-y";
    HistoryLiftedHMembership lifted{
        request.domain.original.relation.metadata()->id, {{0, 2}}, {{0, 4}}};
    request.domain.memberships = {
        {{1},
         {{"prefix"},
          {"prefix"},
          HistorySetFamily::polytope,
          SetRelation::equivalent,
          "x-y",
          {2, 3},
          HistorySourceGeometry{{"prefix"}, HistorySetFamily::polytope, lifted}},
         lifted}};
    auto &start = std::get<SuppliedStart>(request.start);
    start.membership_witnesses = {HistoryHWitness{}};
    auto registration = adapters::nlp::make_casadi_history_optimizer_registration();
    auto workspace = registration.factory(request);
    auto outcome = registry::HistoryOptimizerPort::run(*workspace, request, ExecutionContext{});
    const auto &solved = take<HistoryOptimizationResult>(std::move(outcome));
    CHECK(solved.source_cost.total == doctest::Approx(std::sqrt(2.0)).epsilon(1e-6));
    REQUIRE(solved.membership_witnesses.size() == 1);
    CHECK(std::holds_alternative<HistoryHWitness>(solved.membership_witnesses.front()));
    CHECK(std::get<VerifiedSuppliedStart>(solved.initialization).membership_witness_readback ==
          start.membership_witnesses);
}
