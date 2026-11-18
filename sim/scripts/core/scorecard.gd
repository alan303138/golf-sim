class_name Scorecard
extends RefCounted
## Stroke-play scorecard for 1-4 players.
## Penalty strokes are counted in the hole score and tracked separately.

const MAX_PLAYERS := 4

var players: Array[String] = []
var pars: Array[int] = []
var strokes: Array = []      # [player][hole] -> int
var penalties: Array = []    # [player][hole] -> int
var done: Array = []         # [player][hole] -> bool (holed or picked up)
var picked_up: Array = []    # [player][hole] -> bool
## Pick-up limit: a player stops at par + this many strokes (0 = no limit).
var max_over_par: int = 5
var _tee_orders: Array = []  # [hole] -> Array[int]


func _init(names: Array = ["P1"], hole_pars: Array = [4]) -> void:
	for n in names.slice(0, MAX_PLAYERS):
		players.append(str(n))
	for p in hole_pars:
		pars.append(int(p))
	for i in players.size():
		var s := []
		var pe := []
		var d := []
		var pu := []
		for h in pars.size():
			s.append(0)
			pe.append(0)
			d.append(false)
			pu.append(false)
		strokes.append(s)
		penalties.append(pe)
		done.append(d)
		picked_up.append(pu)


func player_count() -> int:
	return players.size()


func hole_count() -> int:
	return pars.size()


func add_stroke(p: int, h: int, n: int = 1) -> void:
	strokes[p][h] += n


func add_penalty(p: int, h: int, n: int = 1) -> void:
	strokes[p][h] += n
	penalties[p][h] += n


func hole_strokes(p: int, h: int) -> int:
	return strokes[p][h]


func finish_hole(p: int, h: int) -> void:
	done[p][h] = true


func is_done(p: int, h: int) -> bool:
	return done[p][h]


## True when the player has hit the pick-up limit on this hole.
func reached_limit(p: int, h: int) -> bool:
	return max_over_par > 0 and strokes[p][h] >= pars[h] + max_over_par


func pick_up(p: int, h: int) -> void:
	strokes[p][h] = maxi(strokes[p][h], pars[h] + max_over_par)
	picked_up[p][h] = true
	done[p][h] = true


func all_done(h: int) -> bool:
	for p in players.size():
		if not done[p][h]:
			return false
	return true


func total(p: int) -> int:
	var t := 0
	for h in pars.size():
		if done[p][h]:
			t += strokes[p][h]
	return t


func total_par_played(p: int) -> int:
	var t := 0
	for h in pars.size():
		if done[p][h]:
			t += pars[h]
	return t


func to_par(p: int) -> int:
	return total(p) - total_par_played(p)


func course_par() -> int:
	var t := 0
	for x in pars:
		t += x
	return t


## Order on the tee: hole 0 in listed order; afterwards the "honour" goes to
## the lowest score on the previous hole, ties keep the previous order.
func tee_order(h: int) -> Array[int]:
	while _tee_orders.size() <= h:
		var idx := _tee_orders.size()
		var order: Array[int] = []
		if idx == 0:
			for i in players.size():
				order.append(i)
		else:
			var prev: Array = _tee_orders[idx - 1]
			order.assign(prev)
			var sc := strokes
			var prev_h := idx - 1
			# Stable insertion sort by previous hole score.
			for i in range(1, order.size()):
				var j := i
				while j > 0 and sc[order[j - 1]][prev_h] > sc[order[j]][prev_h]:
					var tmp := order[j - 1]
					order[j - 1] = order[j]
					order[j] = tmp
					j -= 1
		_tee_orders.append(order)
	var out: Array[int] = []
	out.assign(_tee_orders[h])
	return out


## Away player plays first: index of the player farthest from the hole
## among those not finished; -1 when everyone is done.
static func next_to_play(distances: Array, finished: Array) -> int:
	var best := -1
	var best_d := -1.0
	for i in distances.size():
		if finished[i]:
			continue
		if float(distances[i]) > best_d:
			best_d = float(distances[i])
			best = i
	return best


static func score_name_key(hole_strokes_: int, par: int) -> String:
	if hole_strokes_ == 1:
		return "SCORE_ACE"
	var d := hole_strokes_ - par
	match d:
		-3:
			return "SCORE_ALBATROSS"
		-2:
			return "SCORE_EAGLE"
		-1:
			return "SCORE_BIRDIE"
		0:
			return "SCORE_PAR"
		1:
			return "SCORE_BOGEY"
		2:
			return "SCORE_DOUBLE_BOGEY"
	if d < -3:
		return "SCORE_ALBATROSS"
	return "SCORE_OVER"


static func format_to_par(n: int) -> String:
	if n == 0:
		return "E"
	return ("+%d" % n) if n > 0 else str(n)


## Expected number of putts for the auto-putt option, drawn from a simple
## make-probability model (roughly amateur-tour blend).
static func auto_putts(distance_m: float, rng: RandomNumberGenerator) -> int:
	var d := maxf(distance_m, 0.0)
	var p1 := 1.0 / (1.0 + pow(d / 1.6, 1.7))      # make probability
	if rng.randf() < p1:
		return 1
	# Second putt from the leave; leave grows with distance.
	var leave := 0.25 + d * 0.08
	var p2 := 1.0 / (1.0 + pow(leave / 1.6, 1.7))
	if rng.randf() < p2:
		return 2
	return 3
