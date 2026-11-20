#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <csl/optimization/finite_action_parallelotope_minimax.hpp>
#include <doctest/doctest.h>
#include <two_points_fixture.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stop_token>

using namespace csl;
using namespace csl::optimization;

TEST_CASE("large diagonal parallelotope scores agree with a physical corner oracle") {
    std::size_t n = 9;
    SUBCASE("enumerate the odd-dimensional terminal box") {}
    SUBCASE("analytic optimum in 48 dimensions") { n = 48; }

    Matrix a{n, n, Vector(n * n)};
    Matrix b{n, 1, Vector(n, 1.0)};
    Matrix q{n, n, Vector(n * n)};
    Vector linear(n);
    Vector endpoints(2 * n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        a.values[i * n + i] = 1;
        q.values[i * n + i] = 2;
        linear[i] = i % 2 == 0 ? -1 : 1;
        endpoints[n + i] = i % 2 == 0 ? 2 : 1;
    }
    // The first pair is decreasing throughout both boxes, so the
    // maximizing vertex must mix lower and upper endpoints.
    linear[0] = -9;
    linear[1] = -7;
    const SystemModel model{
        {{n, 1, 0}, TimeDomain::discrete, LinearStage{a, b, {n, 0, {}}, Vector(n)}}, {}};
    const auto system = model::snapshot(model, Vector{0, 1});
    REQUIRE(std::holds_alternative<model::LinearSystem>(system));
    const auto vertices = numeric::snapshot(Matrix{2, n, endpoints});
    const auto no_rays = numeric::snapshot(Matrix{0, n, {}});
    const auto no_external = numeric::snapshot_vector(Vector{});
    REQUIRE(std::holds_alternative<numeric::MatrixView>(vertices));
    REQUIRE(std::holds_alternative<numeric::MatrixView>(no_rays));
    REQUIRE(std::holds_alternative<numeric::VectorView>(no_external));
    const auto initial = sets::make_v({{"diagonal-initial"}, n, {}},
        {std::get<numeric::MatrixView>(vertices), std::get<numeric::MatrixView>(no_rays)});
    const auto disturbance = sets::make_singleton({{"zero-external"}, 0, {}},
        std::get<numeric::VectorView>(no_external));
    const auto domain = sets::make_universal({{"diagonal-domain"}, n, {}});
    const auto objective = snapshot(QuadraticTerminal{q, linear, 1.25});
    REQUIRE(std::holds_alternative<sets::SetData>(initial));
    REQUIRE(std::holds_alternative<sets::SetData>(disturbance));
    REQUIRE(std::holds_alternative<sets::SetData>(domain));
    REQUIRE(std::holds_alternative<Objective>(objective));
    const auto safe = std::get<sets::SetData>(domain);
    const FiniteActionMinimaxRequest request{
        {"diagonal-frame"}, {"diagonal-frame/1"}, std::get<model::LinearSystem>(system),
        std::get<sets::SetData>(initial), {1, {{1}, {0}, {0}}},
        {std::get<sets::SetData>(disturbance)}, {safe, safe}, std::get<Objective>(objective)};
    const auto answer = finite_action_parallelotope_minimax(request, {3, 2, 2});
    REQUIRE(std::holds_alternative<FiniteActionParallelotopeMinimaxResult>(answer));
    const auto &result = std::get<FiniteActionParallelotopeMinimaxResult>(answer);
    REQUIRE(result.candidate_search == CandidateSearchCoverage::complete);
    REQUIRE(result.plan_attempts.size() == 3);
    for (const auto &item : result.plan_attempts) {
        REQUIRE(std::holds_alternative<ParallelotopePlanScored>(item));
        const auto &attempt = std::get<ParallelotopePlanScored>(item);
        CHECK(attempt.score.corners_evaluated == 1);
        const bool shifted = attempt.action_indices.front() == 0;
        // In exact frame arithmetic each pair has y in [0,1]^2 after
        // propagation. A regular pair's maximum is 4 at (2,1).
        // The action +1 shifts the physical pair, giving max 12 at (3,2).
        // The decreasing first pair contributes 0 or -14 at its lower corner.
        const double analytic = 1.25 + static_cast<double>(n / 2 - 1) * (shifted ? 12 : 4) +
                                (shifted ? -14 : 0) +
                                (n % 2 != 0 ? (shifted ? 6 : 2) : 0);
        CHECK(attempt.score.terminal_frame_box_corner_maximum ==
              doctest::Approx(analytic).epsilon(1e-12));
        if (n == 9) {
            const auto &final = attempt.score.trace.stages.back();
            const auto &frame = attempt.score.trace.frame;
            double maximum = -std::numeric_limits<double>::infinity();
            for (std::size_t mask = 0; mask < (std::size_t{1} << n); ++mask) {
                Vector point(n);
                for (std::size_t row = 0; row < n; ++row)
                    for (std::size_t col = 0; col < n; ++col)
                        point[row] += frame.values[row * n + col] *
                                      ((mask >> col) & 1 ? final.upper[col] : final.lower[col]);
                const auto cost = evaluate(request.objective, point);
                REQUIRE(std::holds_alternative<double>(cost));
                maximum = std::max(maximum, std::get<double>(cost));
            }
            CHECK(attempt.score.terminal_frame_box_corner_maximum ==
                  doctest::Approx(maximum).epsilon(1e-12));
        }
    }
    REQUIRE(std::holds_alternative<ParallelotopePlanSelected>(result.selection));
    CHECK(std::get<ParallelotopePlanSelected>(result.selection).candidate_attempt_index == 1);
}


TEST_CASE("parallelotope selector bounds the exhaustive fallback for large frames") {
    constexpr std::size_t n = 48;
    const SystemModel model{
        {{n, 1, 0},
         TimeDomain::discrete,
         LinearStage{{n, n, Vector(n * n)}, {n, 1, Vector(n)}, {n, 0, {}}, Vector(n)}},
        {}};
    const auto snapped = csl::model::snapshot(model, Vector{0, 1});
    REQUIRE(std::holds_alternative<model::LinearSystem>(snapped));
    const auto point = numeric::snapshot_vector(Vector(n));
    REQUIRE(std::holds_alternative<numeric::VectorView>(point));
    const auto origin =
        sets::make_singleton({{"coupled-initial"}, n, {}}, std::get<numeric::VectorView>(point));
    const auto empty_vector = numeric::snapshot_vector(Vector{});
    REQUIRE(std::holds_alternative<numeric::VectorView>(empty_vector));
    const auto disturbance = sets::make_singleton({{"no-disturbance"}, 0, {}},
                                                  std::get<numeric::VectorView>(empty_vector));
    const auto domain = sets::make_universal({{"coupled-domain"}, n, {}});
    REQUIRE(std::holds_alternative<sets::SetData>(origin));
    REQUIRE(std::holds_alternative<sets::SetData>(disturbance));
    REQUIRE(std::holds_alternative<sets::SetData>(domain));
    auto hessian = Matrix{n, n, Vector(n * n)};
    hessian.values[0] = 2.0;
    hessian.values[2] = 2.0;
    hessian.values[2 * n] = 2.0;
    hessian.values[2 * n + 2] = 2.0;
    const auto objective = snapshot(QuadraticTerminal{hessian, Vector(n), 0});
    REQUIRE(std::holds_alternative<Objective>(objective));
    const auto safe = std::get<sets::SetData>(domain);
    const FiniteActionMinimaxRequest request{{"coupled-score"},
                                             {"coupled-score/1"},
                                             std::get<model::LinearSystem>(snapped),
                                             std::get<sets::SetData>(origin),
                                             {1, {{0.0}}},
                                             {std::get<sets::SetData>(disturbance)},
                                             {safe, safe},
                                             std::get<Objective>(objective)};
    // The large problem must produce a bounded, explicit coverage result
    // before the time allowed for an accidental exhaustive loop expires.
    ExecutionContext context;
    context.deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    const auto answer = finite_action_parallelotope_minimax(request, {1, 1, 1}, context);
    REQUIRE(std::holds_alternative<FiniteActionParallelotopeMinimaxResult>(answer));
    const auto &result = std::get<FiniteActionParallelotopeMinimaxResult>(answer);
    REQUIRE(result.plan_attempts.size() == 1);
    REQUIRE(std::holds_alternative<ParallelotopePlanUnscored>(result.plan_attempts[0]));
    const auto &attempt = std::get<ParallelotopePlanUnscored>(result.plan_attempts[0]);
    CHECK(attempt.outcome.diagnostics.front().field == "finite_parallelotope_minimax.corners");
}

TEST_CASE("TwoPoints parallelotope selector scores each plan and independently "
          "replays the selected plan") {
  const auto request =
      csl::issue824::fixture::two_points_request("parallelotope-minimax-test");
  const auto answer =
      finite_action_parallelotope_minimax(request, {27, 27, 27});
  REQUIRE(
      std::holds_alternative<FiniteActionParallelotopeMinimaxResult>(answer));
  const auto &result = std::get<FiniteActionParallelotopeMinimaxResult>(answer);
  REQUIRE(result.candidate_search == CandidateSearchCoverage::complete);
  REQUIRE(result.plan_attempts.size() == 27);
  std::size_t scored = 0;
  for (const auto &attempt : result.plan_attempts) {
    REQUIRE(std::holds_alternative<ParallelotopePlanScored>(attempt));
    const auto &score = std::get<ParallelotopePlanScored>(attempt).score;
    CHECK(std::isfinite(score.terminal_frame_box_corner_maximum));
    CHECK(score.corners_evaluated == 16);
    CHECK(score.trace.stages.size() == request.system.horizon() + 1);
    ++scored;
  }
  CHECK(scored == 27);
  REQUIRE(std::holds_alternative<ParallelotopePlanSelected>(result.selection));
  const auto &selected = std::get<ParallelotopePlanSelected>(result.selection);
  CHECK(selected.candidate_attempt_index < result.plan_attempts.size());
  CHECK(selected.replay.path.size() == request.system.horizon() + 1);
  CHECK(std::isfinite(selected.replay.worst_value));
}

TEST_CASE("parallelotope selector preserves a stopped candidate and incomplete "
          "coverage") {
  const auto request =
      csl::issue824::fixture::two_points_request("parallelotope-stopped-test");
  std::stop_source cancellation;
  cancellation.request_stop();
  ExecutionContext context;
  context.cancellation = cancellation.get_token();
  const auto answer =
      finite_action_parallelotope_minimax(request, {27, 27, 27}, context);
  REQUIRE(
      std::holds_alternative<FiniteActionParallelotopeMinimaxResult>(answer));
  const auto &result = std::get<FiniteActionParallelotopeMinimaxResult>(answer);
  CHECK(result.candidate_search == CandidateSearchCoverage::incomplete);
  REQUIRE(result.plan_attempts.size() == 1);
  REQUIRE(std::holds_alternative<ParallelotopePlanStopped>(
      result.plan_attempts.front()));
  CHECK(std::get<ParallelotopePlanStopped>(result.plan_attempts.front())
            .outcome.reason == StopReason::cancelled);
}

TEST_CASE("parallelotope selector refuses candidate propagation over the point "
          "budget") {
  const auto request = csl::issue824::fixture::two_points_request(
      "parallelotope-point-limit-test");
  const auto answer = finite_action_parallelotope_minimax(request, {27, 1, 27});
  REQUIRE(
      std::holds_alternative<FiniteActionParallelotopeMinimaxResult>(answer));
  const auto &result = std::get<FiniteActionParallelotopeMinimaxResult>(answer);
  CHECK(result.candidate_search == CandidateSearchCoverage::incomplete);
  REQUIRE(result.plan_attempts.size() == 27);
  for (const auto &attempt : result.plan_attempts) {
    REQUIRE(std::holds_alternative<ParallelotopePlanUnscored>(attempt));
    CHECK(std::get<ParallelotopePlanUnscored>(attempt)
              .outcome.diagnostics.front()
              .field == "finite_parallelotope_minimax.points.limit");
  }
}
