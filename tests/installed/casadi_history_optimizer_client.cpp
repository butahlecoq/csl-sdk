#include <csl/adapters/nlp/casadi_history_optimizer.hpp>

int main() {
    const auto plain = csl::adapters::nlp::make_casadi_history_optimizer_registration();
    const auto scaled = csl::adapters::nlp::make_casadi_scaled_history_optimizer_registration(
        {{1000.0, 0.001}, {0.5}});
    const auto mumps = csl::adapters::nlp::make_casadi_mumps_history_optimizer_registration();
    return plain.identity.id.value == "csl.nlp.casadi-ipopt.history-cost" &&
                   scaled.identity.id.value == "csl.nlp.casadi-ipopt.scaled-history-cost" &&
                   mumps.identity.id.value == "csl.nlp.casadi-ipopt.mumps-history-cost"
               ? 0
               : 1;
}
