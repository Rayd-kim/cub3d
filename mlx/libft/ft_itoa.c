/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_itoa.c                                          :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/11/17 17:14:15 by youskim           #+#    #+#             */
/*   Updated: 2021/11/18 13:09:38 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include <stdlib.h>

static char	*ch_num(char *str, int count, unsigned int num, int sign)
{
	int	i;

	i = 0;
	if (sign == 0)
	{
		while (i < count)
		{
			str[count - i - 1] = num % 10 + '0';
			num = num / 10;
			i++;
		}
		str[count] = '\0';
	}
	else
	{
		str[0] = '-';
		while (i < count)
		{
			str[count - i] = num % 10 + '0';
			num = num / 10;
			i++;
		}
		str[count + 1] = '\0';
	}
	return (str);
}

static int	num_count(unsigned int n)
{
	int	count;

	count = 1;
	while (n >= 10)
	{
		count++;
		n = n / 10;
	}
	return (count);
}

char	*ft_itoa(int n)
{
	int				count;
	int				sign;
	unsigned int	num1;
	char			*arr;

	sign = 0;
	if (n < 0)
	{
		sign = 1;
		num1 = -n;
	}
	else
		num1 = n;
	count = num_count(num1);
	arr = (char *)malloc(sizeof(char) * (count + sign + 1));
	if (arr == 0)
		return (NULL);
	arr = ch_num(arr, count, num1, sign);
	return (arr);
}
