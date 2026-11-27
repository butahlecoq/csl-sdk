"""Exercise the native history strategy path through the line-delimited bridge."""

import json
import math
import subprocess
import sys

request = {
    "schema": "csl.closed-loop.native-policy-query/1",
    "mode": "continuous-history/1",
    "history_solver": "casadi-ipopt",
    "geometry_family": "polytope",
    "polytope_history": "lifted_whole_history",
    "task_id": "native-history-feedback-test",
    "task_sha256": "a" * 64,
    "source_model_id": "integrator",
    "controller_id": "native-history-feedback-test/1",
    "action_profile_id": "original-continuous-actions",
    "problem_id": "native-history-feedback-test",
    "request_id": "update-0",
    "absolute_update_index": 0,
    "observed_physical_state": [0.0, 0.0],
    "original_horizon": 1,
    "dt": 1.0,
    "predictor_stages": [
        {"A": [[1.0, 0.0], [0.0, 1.0]], "B": [[1.0], [0.0]], "c": [0.0, 0.0]}
    ],
    "state_bounds": [[-2.0, 2.0], [-2.0, 2.0]],
    "control_bounds": [[-1.0, 1.0]],
    "terminal_objective": {
        "kind": "quadratic",
        "Q": [[2.0, 0.0], [0.0, 0.0]],
        "linear": [-1.0, 0.0],
        "offset": 0.25,
    },
    "running_cost": {
        "objective_id": "test-nonzero-running-cost",
        "stages": [
            {
                "stage": 0,
                "terms": [
                    {
                        "kind": "quadratic",
                        "reference": [0.0],
                        "scales": [1.0],
                        "weights": [1.0],
                        "coefficient": 1.0,
                    }
                ],
            }
        ],
    },
    "initial_primal": [0.0] * 5,
    "initial_geometry_factors": [[]] * 6,
    "native_history_optimizer": "csl.nlp.casadi-ipopt.history-cost",
}

requests = [
    dict(request, native_history_strategy="A"),
    dict(request, native_history_strategy="B"),
    dict(request, native_history_strategy="B", initial_geometry_factors=[[]]),
    dict(request, native_history_strategy="other"),
    dict(
        request, native_history_strategy="A", initial_primal=[0.0, 0.0, 0.0, 0.0, 0.5]
    ),
    {
        key: value
        for key, value in request.items()
        if key not in {"native_history_optimizer", "initial_geometry_factors"}
    },
]


def exchange(requests):
    run = subprocess.run(
        [sys.argv[1]],
        input="".join(json.dumps(item) + "\n" for item in requests),
        text=True,
        capture_output=True,
        check=True,
        timeout=90,
    )
    responses = [json.loads(line) for line in run.stdout.splitlines()]
    assert len(responses) == len(requests), run.stdout
    return responses


responses = exchange(requests)
for strategy, response in zip(("A", "B"), responses[:2], strict=True):
    assert response["status"] == "solved", response
    assert response["task_sha256"] == request["task_sha256"]
    receipt = response["solver_receipt"]
    assert receipt["provider"] == request["native_history_optimizer"]
    assert receipt["strategy"] == strategy
    assert receipt["supplied_readback"]
    assert receipt["geometry_factors"] == [[]] * 6
    assert math.isclose(response["primal"][-1], 0.25, abs_tol=1e-7)
    assert math.isclose(receipt["objective"], 0.125, abs_tol=1e-7)
for response in responses[2:4]:
    assert response["status"] == "rejected", response
    assert "solver_receipt" not in response
assert responses[4]["status"] == "failed", responses[4]
assert "solver_receipt" not in responses[4]
assert responses[5]["status"] == "exported", responses[5]


for family in ("box", "parallelotope", "zonotope", "ellipsoid"):
    options = dict(
        request,
        geometry_family=family,
        polytope_history="stagewise",
        geometry_max_generators=32,
    )
    [export] = exchange([options])
    assert export["status"] == "exported", export
    assert export["geometry"]["polytope_history"] == "stagewise", export
    assert export["geometry"]["route"] == "stagewise", export
    members = [
        member
        for stage in export["geometry"]["stages"]
        for member in stage["memberships"]
    ]
    seed = [[0.0] * member.get("factor_count", 0) for member in members]
    provider = "csl.nlp.casadi-ipopt.mumps-history-cost"
    solves = exchange(
        [
            dict(
                options,
                native_history_strategy=strategy,
                native_history_optimizer=provider,
                initial_geometry_factors=seed,
            )
            for strategy in ("A", "B")
        ]
    )
    for strategy, result in zip(("A", "B"), solves, strict=True):
        assert result["status"] == "solved", result
        assert result["solver_receipt"]["supplied_readback"]
        assert result["solver_receipt"]["provider"] == provider
        assert len(result["solver_receipt"]["geometry_factors"]) == len(seed)
        assert math.isclose(result["primal"][-1], 0.25, abs_tol=1e-7)

        bindings = result["solver_receipt"]["geometry_bindings"]
        expected = [
            dict(
                stage=stage["stage"],
                role=member["role"],
                selected=member["set_id"],
                reference=stage[member["role"] + "_h"]["set_id"],
                relation=member["relation"],
            )
            for stage in result["geometry"]["stages"]
            for member in stage["memberships"]
        ]
        assert bindings == expected
        assert any(
            item["selected"] != item["reference"] and item["relation"] == "outer"
            for item in bindings
        )
        if family in {"box", "parallelotope"}:
            [round_trip] = exchange(
                [
                    dict(
                        options,
                        native_history_strategy=strategy,
                        native_history_optimizer=provider,
                        initial_primal=result["primal"],
                        initial_geometry_factors=result["solver_receipt"][
                            "geometry_factors"
                        ],
                    )
                ]
            )
            assert round_trip["status"] == "solved", round_trip

certified = dict(
    request,
    geometry_family="polytope",
    polytope_history="certified_stagewise",
    geometry_max_generators=32,
)
[certified_export] = exchange([certified])
assert certified_export["status"] == "exported", certified_export
certified_geometry = certified_export["geometry"]
assert certified_geometry["polytope_history"] == "certified_stagewise", certified_geometry
assert (
    certified_geometry["route"] == "certified_stagewise_projection"
), certified_geometry
assert certified_geometry["stages"], certified_geometry
for stage in certified_geometry["stages"]:
    for member in stage["memberships"]:
        assert member["relation"] == "outer", member
        assert member["assurance"] == "numerical", member

orbit = dict(
    request,
    task_id="env717-earth-orbit/continuous-receding/1",
    native_history_strategy="A",
)
[bad_observation, invalid_warm_start, invalid_solution] = exchange(
    [
        orbit,
        dict(
            orbit,
            observed_physical_state=[0.5, 0.0],
            initial_primal=[0.5, 0.0, -0.5, 0.0, -1.0],
        ),
        dict(
            orbit,
            observed_physical_state=[0.5, 0.0],
            initial_primal=[0.5, 0.0, 0.5, 0.0, 0.0],
            terminal_objective={
                "kind": "quadratic",
                "Q": [[2.0, 0.0], [0.0, 0.0]],
                "linear": [2.0, 0.0],
                "offset": 1.0,
            },
        ),
    ]
)
assert (
    bad_observation["status"] == "rejected"
    and "Observation violates" in bad_observation["reason"]
), bad_observation
assert (
    invalid_warm_start["status"] == "rejected"
    and "Supplied history violates" in invalid_warm_start["reason"]
), invalid_warm_start
assert (
    invalid_solution["status"] == "failed"
    and "Returned history violates" in invalid_solution["reason"]
), invalid_solution
assert "solver_receipt" in invalid_solution and "primal" not in invalid_solution
