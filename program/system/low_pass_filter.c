float low_pass_filter(float new, float old, float alpha)
{
	return alpha * new + (1 - alpha) * old;
}
