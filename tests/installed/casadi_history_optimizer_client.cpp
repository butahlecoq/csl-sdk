#include <csl/adapters/nlp/casadi_history_optimizer.hpp>

int main() {
    const auto plain = csl::adapters::nlp::make_casadi_history_optimizer_registration();
    const auto scaled = csl::adapters::nlp::make_casadi_scaled_history_optimizer_registration(
        {{1000.0, 0.001}, {0.5}});
    return plain.identity.id.value == "csl.nlp.casadi-ipopt.history-cost" &&
                   scaled.identity.id.value == "csl.nlp.casadi-ipopt.scaled-history-cost"
               ? 0
               : 1;
}
