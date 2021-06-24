function final_ans = choose_ans(ans,type)
ans1 = double(ans(1));
ans2 = double(ans(2));
if type == 'rot'
    if ans1<=pi && ans1>=0
        final_ans = ans1
    else
        final_ans = ans2
    end
elseif type == 'trans'
    if ans1<=300 && ans1>=0
        final_ans = ans1
    else
        final_ans = ans2
    end
end