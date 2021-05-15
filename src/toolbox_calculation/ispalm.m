function flag = ispalm(idxF, idxL)
	% idxF: finger index
	% idxL: link index
	if nargin < 2
		flag = idxF == 0;
		return;
	end

	if (idxF==0) ~= (idxL==0)
		warning('Invalid finger or link index.');
	end

	flag = (idxF==0) || (idxL==0);
end