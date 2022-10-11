/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   ft_substr.c                                        :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2021/11/17 16:09:31 by youskim           #+#    #+#             */
/*   Updated: 2021/11/21 23:42:24 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#include "libft.h"

static size_t	check_len(char const *s, unsigned int start, size_t len)
{
	size_t			k;
	unsigned int	temp;

	k = 0;
	temp = start;
	while (s[temp] != '\0' && start < ft_strlen(s))
	{
		temp++;
		k++;
	}
	if (len <= k)
		return (len);
	else
		return (k);
}

char	*ft_substr(char const *s, unsigned int start, size_t len)
{
	char	*arr;
	size_t	i;
	size_t	k;

	k = check_len(s, start, len);
	arr = (char *)malloc(sizeof(char) * (k + 1));
	if (arr == 0)
		return (NULL);
	if (start >= ft_strlen(s))
	{
		arr[0] = '\0';
		return (arr);
	}
	i = 0;
	while (s[start] != '\0' && i < len)
	{
		arr[i] = s[start];
		i++;
		start++;
	}
	arr[i] = '\0';
	return (arr);
}
