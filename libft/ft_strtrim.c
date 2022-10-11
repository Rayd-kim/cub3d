/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_strtrim.c                                       :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/11/17 16:28:26 by youskim           #+#    #+#             */
/*   Updated: 2021/11/24 21:05:50 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static int	check_set(char c, char const *set)
{
	size_t	i;

	i = 0;
	while (i < ft_strlen(set))
	{
		if (c == set[i])
			return (1);
		i++;
	}
	return (0);
}

static int	first_len(char const *s1, char const *set)
{
	size_t	i;

	i = 0;
	while (check_set(s1[i], set) == 1)
		i++;
	return (i);
}

static int	end_len(char const *s1, char const *set)
{
	size_t	k;
	int		count;

	k = ft_strlen(s1);
	count = 0;
	if (k == 0)
		return (count);
	else
	{
		while (check_set(s1[k - 1], set) == 1)
		{
			count++;
			k--;
		}
	}
	return (count);
}

char	*ft_strtrim(char const *s1, char const *set)
{
	char		*arr;
	int			count;
	int			i;
	size_t		first;
	size_t		end;

	first = first_len(s1, set);
	if (first == ft_strlen(s1))
		count = 0;
	else
	{
		end = end_len(s1, set);
		count = ft_strlen(s1) - first - end;
	}
	arr = (char *)malloc(sizeof(char) * (count + 1));
	if (arr == 0)
		return (NULL);
	i = -1;
	while (++i < count)
	{
		arr[i] = s1[first];
		first++;
	}
	arr[i] = '\0';
	return (arr);
}
