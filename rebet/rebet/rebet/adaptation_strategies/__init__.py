from rebet.adaptation_strategies.random_adaptation_strategy import RandomStrategy
from rebet.adaptation_strategies.bandit_adaptation_strategy import BanditStrategy
from rebet.adaptation_strategies.suave_strategies import suave_strategies

def create_strategy(strategy_name):
    online_strategies = {
        "random_strategy": RandomStrategy,
        "ucb_strategy": (BanditStrategy, "UCB")
    }

    all_strategies = {**online_strategies, **suave_strategies}
    
    chosen_strategy = all_strategies.get(strategy_name, None)

    if(chosen_strategy is not None):
        if(type(chosen_strategy) is tuple): return chosen_strategy[0](*chosen_strategy[1:])
        else: return chosen_strategy()
    else:
        raise RuntimeError("Specified Strategy " + strategy_name + " did not exist")
