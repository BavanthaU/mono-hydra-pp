import kimera_rpgo


def cost_from_str(cost_str):
    if (
        cost_str.lower() == "none"
        or cost_str.lower() == "ls"
        or cost_str.lower() == "quadratic"
    ):
        return kimera_rpgo.LossType.LS
    if cost_str.lower() == "huber":
        return kimera_rpgo.LossType.HUBER
    if cost_str.lower() == "gm":
        return kimera_rpgo.LossType.GM
    if cost_str.lower() == "tukey":
        return kimera_rpgo.LossType.TUKEY
    if cost_str.lower() == "tls" or cost_str.lower().startswith("truncate"):
        return kimera_rpgo.LossType.TLS
